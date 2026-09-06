/*******************************************************************
 * Unified Dynamixel interface.                                    *
 *                                                                  *
 * A single class that talks to all the servos of the robot, with  *
 * the command modality (current vs. turns) selected once at       *
 * construction time. Feedback (turns + current/torque) is always  *
 * available regardless of the selected command mode, since the    *
 * present-position and present-current control-table registers    *
 * are readable in any operating mode.                             *
 *******************************************************************/
#ifndef DYNAMIXEL_INTERFACE_H_
#define DYNAMIXEL_INTERFACE_H_

#include "dynamixel_interface/dynamixel_utils.h"
#include <vector>

// --- Command Modality --- //
// CURRENT           - direct current/torque control (CURRENT_MODE).
// TURNS             - position control by turn count (EXTENDED_POSITION_MODE), no torque ceiling.
// CURRENT_POSITION  - position control by turn count, with a fixed torque ceiling
//                     (CURRENT_POSITION_MODE) - compliant position control, e.g. to
//                     avoid overtensioning a tendon on contact/stall.
enum class CommandMode { CURRENT, TURNS, CURRENT_POSITION };

class DynamixelInterface
{
    // --- Protected Attributes --- //
    protected:
        // Communication Utils
        PortHandler *portHandler        = PortHandler::getPortHandler(DEVICE_NAME);
        PacketHandler *packetHandler    = PacketHandler::getPacketHandler(BAUDRATE);

        // N° of motors
        int n_motors = 0;

        // Command Modality (fixed for the object's lifetime; the op-mode register
        // is only ever programmed once, at construction)
        CommandMode mode;

        // Error Handling
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Sync Write objects: both position and current are always instantiated,
        // since only the write path matching `mode` is ever used but either
        // could be needed (e.g. by the destructor's shutdown sequence).
        GroupSyncWrite position_syncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, POSITION_BYTE);
        GroupSyncWrite current_syncWrite  = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, CURRENT_BYTE);

        // Single sync read covering current+velocity+position together (see
        // FEEDBACK_BYTE_LENGTH) - one bus transaction serves every feedback
        // getter below, regardless of which fields the caller actually wants.
        GroupSyncRead feedback_syncRead = GroupSyncRead(portHandler, packetHandler, FEEDBACK_START_ADDR, FEEDBACK_BYTE_LENGTH);

        // Zero-turn reference, latched at construction and re-latched by enableTorque()
        std::vector<int32_t> initial_positions;

        // Per-motor bring-up status (did every init write for this motor succeed?)
        std::vector<bool> motor_ready;

        // Scratch buffers reused across calls so steady-state reads/writes do
        // not heap-allocate on every control tick.
        std::vector<int32_t> position_registers_buf;
        std::vector<int16_t> current_registers_buf;
        std::vector<uint8_t> position_write_buffer;
        std::vector<uint8_t> current_write_buffer;

        // --- Low Level Helpers --- //
        bool writePositions(const std::vector<int32_t>& registers);
        bool writeCurrents(const std::vector<int16_t>& registers);
        // Fills position_registers_buf / current_registers_buf from one combined sync read.
        bool readFeedbackRegisters();
        // Shared by set_command()'s CURRENT branch and set_torques(): converts a
        // per-motor amps array to registers (with saturation) and writes it.
        bool commandCurrents(const std::vector<float>& amps);

    // Methods
    public:
        // --- Constructor / Destructor --- //
        // current_limit_amps is only meaningful for CommandMode::CURRENT_POSITION,
        // where it's written once (as Goal Current, a RAM register) as the torque
        // ceiling the position controller is allowed to use; ignored otherwise.
        DynamixelInterface(int n_motors, CommandMode mode, float current_limit_amps = MAX_CURRENT);
        ~DynamixelInterface();

        // Did every motor come up correctly at construction?
        bool allMotorsReady() const;
        CommandMode getMode() const { return mode; }

        // --- Command (dispatches to current or turns internally, per `mode`) --- //
        bool set_command(const std::vector<float>& cmd);

        // Torque command [Nm], converted via the COEFF_*-fit torque2Current/torque2Register
        // in dynamixel_utils.h. Only valid in CommandMode::CURRENT (fails otherwise, since
        // TURNS/CURRENT_POSITION don't drive the current register directly per command).
        bool set_torques(const std::vector<float>& torques);

        // --- Feedback: always available, independent of the command mode --- //
        // Prefer get_feedback() when both are needed (the common case): it costs
        // one bus transaction, whereas calling get_turns()+get_currents() costs two.
        bool get_turns(std::vector<float>& turns);
        bool get_currents(std::vector<float>& currents);
        bool get_torques(std::vector<float>& torques);
        bool get_feedback(std::vector<float>& turns, std::vector<float>& currents);

        // --- Torque control --- //
        bool enableTorque();     // also re-latches the zero-turn reference
        bool disableTorque();
        bool update_initPos();
};

#endif /* DYNAMIXEL_INTERFACE_H_ */
