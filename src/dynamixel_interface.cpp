/****************************************
 * Source code for the unified          *
 * Dynamixel interface class            *
 ****************************************/
#include "dynamixel_interface/dynamixel_interface.h"

namespace
{
    // Logs a comm-result / hardware-error failure for one motor and returns
    // whether the write/read actually succeeded. Shared by every per-motor
    // register access below so failure handling stays consistent in one place.
    bool checkResult(int dxl_comm_result, uint8_t dxl_error, int motor_id, const char* action)
    {
        if(dxl_comm_result != COMM_SUCCESS)
        {
            ROS_ERROR("Failed to %s for Dynamixel ID %d (comm result: %d)", action, motor_id, dxl_comm_result);
            return false;
        }
        if(dxl_error != 0)
        {
            ROS_WARN("Dynamixel ID %d reported a hardware error (code %d) while trying to %s", motor_id, dxl_error, action);
            return false;
        }
        return true;
    }
}

// --- Constructor --- //
DynamixelInterface::DynamixelInterface(int n_dyna, CommandMode command_mode, float current_limit_amps,
                                        float max_turns_val, uint32_t profile_velocity, uint32_t profile_acceleration)
{
    n_motors = n_dyna;
    mode = command_mode;
    max_turns = max_turns_val;
    motor_ready.assign(n_motors, false);

    uint8_t op_mode;
    switch(mode)
    {
        case CommandMode::CURRENT:          op_mode = CURRENT_MODE;            break;
        case CommandMode::CURRENT_POSITION: op_mode = CURRENT_POSITION_MODE;   break;
        case CommandMode::TURNS:
        default:                            op_mode = EXTENDED_POSITION_MODE;  break;
    }

    // Turn On LED | Op Mode | Enable Torque | Profile Velocity | Profile Acceleration
    // Every motor gets a full attempt regardless of an earlier motor's failure,
    // so one unresponsive servo does not leave the rest of the arm unconfigured.
    for(int i = 0; i < n_motors; i++)
    {
        uint8_t id = i + 1;
        bool ok = true;

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_LED, LED_ON, &dxl_error);
        ok &= checkResult(dxl_comm_result, dxl_error, id, "turn on LED");

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OP_MODE, op_mode, &dxl_error);
        ok &= checkResult(dxl_comm_result, dxl_error, id, "set operating mode");

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        ok &= checkResult(dxl_comm_result, dxl_error, id, "enable torque");

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VEL, profile_velocity, &dxl_error);
        ok &= checkResult(dxl_comm_result, dxl_error, id, "set profile velocity");

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACC, profile_acceleration, &dxl_error);
        ok &= checkResult(dxl_comm_result, dxl_error, id, "set profile acceleration");

        motor_ready[i] = ok;
    }

    if(!allMotorsReady())
        ROS_ERROR("Not all Dynamixels initialized correctly - check wiring/IDs/power before commanding the robot.");

    // Preallocate scratch/write buffers once so steady-state operation never
    // heap-allocates per call.
    position_registers_buf.reserve(n_motors);
    current_registers_buf.reserve(n_motors);
    position_write_buffer.assign(n_motors * POSITION_BYTE, 0);
    current_write_buffer.assign(n_motors * CURRENT_BYTE, 0);

    // Current-based Position mode: latch the torque ceiling once, as Goal
    // Current (a RAM register - no torque-disable/EEPROM dance needed). The
    // position controller uses this as a hard cap on how hard it can push to
    // reach a commanded turn count, e.g. to avoid overtensioning a tendon.
    if(mode == CommandMode::CURRENT_POSITION)
    {
        int16_t limit_reg = current2Register(current_limit_amps);
        if(!registerCur_saturation(limit_reg))
            ROS_WARN("Requested current limit (%.3f A) is out of range; saturating to +/-%.3f A.", current_limit_amps, MAX_CURRENT);

        if(!writeCurrents(std::vector<int16_t>(n_motors, limit_reg)))
            ROS_ERROR("Failed to set the current (torque) ceiling for Current-based Position mode.");
    }

    // Latch the "zero turns" reference to wherever the motors physically are now
    if(!readFeedbackRegisters())
        ROS_ERROR("Failed to read initial positions of the motors.");
    else
        initial_positions = position_registers_buf;
}

// --- Destructor --- //
// No template/virtual-dispatch base class involved: this destructor runs as
// the concrete, most-derived type, so calling the real shutdown logic directly
// here (rather than through a virtual call from a base class destructor) is safe.
DynamixelInterface::~DynamixelInterface()
{
    ROS_WARN("Terminating Dynamixel interface...");

    bool shutdown_ok;
    if(mode == CommandMode::CURRENT)
        shutdown_ok = writeCurrents(std::vector<int16_t>(n_motors, 0));
    else
        shutdown_ok = writePositions(initial_positions);

    if(!shutdown_ok)
        ROS_ERROR("Failed to zero all motors before shutdown.");

    if(!disableTorque())
        ROS_ERROR("Failed to disable torque/LED on all motors during shutdown.");

    ROS_WARN("Dynamixel interface terminated.");
}

bool DynamixelInterface::allMotorsReady() const
{
    for(bool ready : motor_ready)
        if(!ready)
            return false;
    return true;
}

// --- Low Level Helpers --- //
// position_write_buffer/current_write_buffer are sized once (in the
// constructor) to n_motors*BYTE_WIDTH and reused here on every call: no
// heap allocation on the steady-state command path. Each motor's segment is
// fully overwritten before addParam() (which the SDK copies out of
// immediately), so reusing the buffer across calls is safe.
bool DynamixelInterface::writePositions(const std::vector<int32_t>& registers)
{
    for(int i = 0; i < n_motors; i++)
    {
        uint8_t* param = &position_write_buffer[i * POSITION_BYTE];
        param[0] = DXL_LOBYTE(DXL_LOWORD(registers[i]));
        param[1] = DXL_HIBYTE(DXL_LOWORD(registers[i]));
        param[2] = DXL_LOBYTE(DXL_HIWORD(registers[i]));
        param[3] = DXL_HIBYTE(DXL_HIWORD(registers[i]));

        if(!position_syncWrite.addParam((uint8_t)(i + 1), param))
        {
            ROS_ERROR("Failed to addParam to groupSyncWrite for Dynamixel ID %d", i + 1);
            position_syncWrite.clearParam();
            return false;
        }
    }

    dxl_comm_result = position_syncWrite.txPacket();
    position_syncWrite.clearParam();

    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        return false;
    }

    return true;
}

bool DynamixelInterface::writeCurrents(const std::vector<int16_t>& registers)
{
    for(int i = 0; i < n_motors; i++)
    {
        uint8_t* param = &current_write_buffer[i * CURRENT_BYTE];
        param[0] = DXL_LOBYTE(registers[i]);
        param[1] = DXL_HIBYTE(registers[i]);

        if(!current_syncWrite.addParam((uint8_t)(i + 1), param))
        {
            ROS_ERROR("Failed to addParam to groupSyncWrite for Dynamixel ID %d", i + 1);
            current_syncWrite.clearParam();
            return false;
        }
    }

    dxl_comm_result = current_syncWrite.txPacket();
    current_syncWrite.clearParam();

    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
        return false;
    }

    return true;
}

// One sync read spanning current+velocity+position (see FEEDBACK_BYTE_LENGTH)
// instead of two separate transactions. position_registers_buf/
// current_registers_buf are members reused across calls (clear() keeps their
// capacity), so this doesn't heap-allocate past the first call either.
bool DynamixelInterface::readFeedbackRegisters()
{
    for(int id = 1; id <= n_motors; id++)
    {
        if(!feedback_syncRead.addParam((uint8_t) id))
        {
            ROS_ERROR("Failed to addParam to groupSyncRead for Dynamixel ID %d", id);
            feedback_syncRead.clearParam();
            return false;
        }
    }

    dxl_comm_result = feedback_syncRead.txRxPacket();
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to get feedback! Result: %d", dxl_comm_result);
        feedback_syncRead.clearParam();
        return false;
    }

    position_registers_buf.clear();
    current_registers_buf.clear();
    for(int id = 1; id <= n_motors; id++)
    {
        position_registers_buf.push_back(feedback_syncRead.getData((uint8_t) id, ADDR_PRESENT_POSITION, POSITION_BYTE));
        current_registers_buf.push_back((int16_t) feedback_syncRead.getData((uint8_t) id, ADDR_PRESENT_CURRENT, CURRENT_BYTE));
    }

    feedback_syncRead.clearParam();
    return true;
}

// --- Command --- //
bool DynamixelInterface::set_command(const std::vector<float>& cmd)
{
    if((int) cmd.size() != n_motors)
    {
        ROS_ERROR("set_command: expected %d values, got %zu.", n_motors, cmd.size());
        return false;
    }

    if(mode == CommandMode::CURRENT)
        return commandCurrents(cmd);

    // CommandMode::TURNS or CommandMode::CURRENT_POSITION: both are commanded by
    // turn count; CURRENT_POSITION's torque ceiling was already latched once at
    // construction and isn't touched here.
    std::vector<int32_t> registers(n_motors);
    for(int i = 0; i < n_motors; i++)
    {
        float turn = cmd[i];
        if(!turns_saturation(turn, max_turns))
            ROS_WARN("Commanded turns for Dynamixel ID %d is out of limits. Saturating...", i + 1);
        registers[i] = (int32_t)(turn * (float) ONE_TURN_REGISTER) + initial_positions[i];
    }
    return writePositions(registers);
}

bool DynamixelInterface::set_torques(const std::vector<float>& torques)
{
    if(mode != CommandMode::CURRENT)
    {
        ROS_ERROR("set_torques: only valid in CommandMode::CURRENT.");
        return false;
    }

    if((int) torques.size() != n_motors)
    {
        ROS_ERROR("set_torques: expected %d values, got %zu.", n_motors, torques.size());
        return false;
    }

    // Convert torque [Nm] -> current [A] via the fit, then reuse the exact same
    // register-level saturation/write path as a direct current command.
    std::vector<float> amps(n_motors);
    for(int i = 0; i < n_motors; i++)
        amps[i] = torque2Current(torques[i]);

    return commandCurrents(amps);
}

// Shared by set_command()'s CURRENT branch and set_torques(): converts a
// per-motor amps array to registers (with saturation) and writes it.
bool DynamixelInterface::commandCurrents(const std::vector<float>& amps)
{
    std::vector<int16_t> registers(n_motors);
    for(int i = 0; i < n_motors; i++)
    {
        int16_t reg = current2Register(amps[i]);
        if(!registerCur_saturation(reg))
            ROS_WARN("Commanded current for Dynamixel ID %d is out of limits. Saturating...", i + 1);
        registers[i] = reg;
    }
    return writeCurrents(registers);
}

// --- Feedback --- //
// Each of these costs one full combined bus transaction (current+velocity+
// position) even when only one field is needed - simpler than maintaining
// separate narrow reads, and per-instruction overhead dominates the cost of
// the couple of extra unused bytes anyway. Callers that want both turns and
// currents together (the common case - see Ros_Dynamixel_Node) should use
// get_feedback() instead, which pays that one transaction only once.
bool DynamixelInterface::get_turns(std::vector<float>& turns)
{
    if(!readFeedbackRegisters())
        return false;

    turns.resize(n_motors);
    for(int i = 0; i < n_motors; i++)
        turns[i] = register2Turns(position_registers_buf[i], initial_positions[i]);

    return true;
}

bool DynamixelInterface::get_currents(std::vector<float>& currents)
{
    if(!readFeedbackRegisters())
        return false;

    currents.resize(n_motors);
    for(int i = 0; i < n_motors; i++)
        currents[i] = register2Current(current_registers_buf[i]);

    return true;
}

bool DynamixelInterface::get_torques(std::vector<float>& torques)
{
    if(!readFeedbackRegisters())
        return false;

    torques.resize(n_motors);
    for(int i = 0; i < n_motors; i++)
        torques[i] = register2Torque(current_registers_buf[i]);

    return true;
}

bool DynamixelInterface::get_feedback(std::vector<float>& turns, std::vector<float>& currents)
{
    if(!readFeedbackRegisters())
        return false;

    turns.resize(n_motors);
    currents.resize(n_motors);
    for(int i = 0; i < n_motors; i++)
    {
        turns[i]    = register2Turns(position_registers_buf[i], initial_positions[i]);
        currents[i] = register2Current(current_registers_buf[i]);
    }

    return true;
}

// --- Torque Control --- //
bool DynamixelInterface::update_initPos()
{
    // Reuses the exact same 0-indexed read path as construction, so the
    // zero-turn reference can never drift out of alignment with motor IDs.
    if(!readFeedbackRegisters())
        return false;

    initial_positions = position_registers_buf;
    return true;
}

bool DynamixelInterface::enableTorque()
{
    bool all_ok = true;
    for(int i = 0; i < n_motors; i++)
    {
        uint8_t id = i + 1;

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        all_ok &= checkResult(dxl_comm_result, dxl_error, id, "enable torque");

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_LED, LED_ON, &dxl_error);
        all_ok &= checkResult(dxl_comm_result, dxl_error, id, "turn on LED");
    }

    // Re-latch the zero-turn reference to wherever the motors physically are now
    if(!update_initPos())
    {
        ROS_ERROR("Failed to update initial positions of the motors.");
        all_ok = false;
    }

    return all_ok;
}

bool DynamixelInterface::disableTorque()
{
    bool all_ok = true;
    for(int i = 0; i < n_motors; i++)
    {
        uint8_t id = i + 1;

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        all_ok &= checkResult(dxl_comm_result, dxl_error, id, "disable torque");

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_LED, LED_OFF, &dxl_error);
        all_ok &= checkResult(dxl_comm_result, dxl_error, id, "turn off LED");
    }

    return all_ok;
}
