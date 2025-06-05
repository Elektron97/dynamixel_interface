#include "dynamixel_interface/extpos_dynamixel.h"

// --- Extended Position Dynamixel Class --- //
ExtPos_Dynamixel::ExtPos_Dynamixel(int n_motors)
{
    // Init n_motors
    this->n_motors = n_motors;

    // Error Handling
    // Init done by Super Class Constructor
    //dxl_error = 0;
    //dxl_comm_result = COMM_TX_FAIL;

    // Turn On LED | Current Mode | Enable Torque | Profile velocity | Profile Acceleration
    i = 0;
    for(i; i < this->n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn on LED for Dynamixel ID %d", i+1);
            break;
        }

        // Extended-Position Drive Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_OP_MODE, EXTENDED_POSITION_MODE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set Extended Position Mode for Dynamixel ID %d", i+1);
            break;
        }

        // Enable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
            break;
        }

        // Profile Velocity
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, ADDR_PROFILE_VEL, PROFILE_VEL_VALUE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set profile velocity for Dynamixel ID %d", i+1);
            break;
        }
        
        // Profile Acceleration
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, ADDR_PROFILE_ACC, PROFILE_ACC_VALUE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set profile acceleration for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Init Motors Mask
    motors_mask = std::vector<bool>(this->n_motors, true);

    // Read Initial Position
    if(!get_PosRegisters(initial_positions))
        ROS_ERROR("Failed to read all initial positions of the motors.");
}

bool ExtPos_Dynamixel::get_PosRegisters(std::vector<int32_t>& positions)
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add all motors' id
    i = 1;
    for(i; i <= n_motors; i++)
    {
        // Supposing that Motors ID are 1, 2, 3, 4, ..., n_motors
        dxl_addparam_result = position_syncRead.addParam((uint8_t) i);
        if (dxl_addparam_result != true) 
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", i);
            break;
        }
    }

    // Read all motors
    dxl_comm_result = position_syncRead.txRxPacket();
    if(dxl_comm_result == COMM_SUCCESS)
    {
        for(i = 1; i <= n_motors; i++)
        {
            positions.push_back(position_syncRead.getData((uint8_t) i, ADDR_PRESENT_POSITION, POSITION_BYTE));
        }

        position_syncRead.clearParam();
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get positions! Result: %d", dxl_comm_result);
        position_syncRead.clearParam();
        return false;       
    }
}

bool ExtPos_Dynamixel::get_CurRegisters(std::vector<int16_t>& currents)
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add all motors' id
    i = 1;
    for(i; i <= n_motors; i++)
    {
        // Supposing that Motors ID are 1, 2, 3, 4, ..., n_motors
        dxl_addparam_result = current_syncRead.addParam((uint8_t) i);
        if (dxl_addparam_result != true) 
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", i);
            break;
        }
    }

    // Read all motors
    dxl_comm_result = current_syncRead.txRxPacket();
    if(dxl_comm_result == COMM_SUCCESS)
    {
        for(i = 1; i <= n_motors; i++)
        {
            currents.push_back(current_syncRead.getData((uint8_t) i, ADDR_PRESENT_CURRENT, CURRENT_BYTE));
        }

        current_syncRead.clearParam();
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get currents! Result: %d", dxl_comm_result);
        current_syncRead.clearParam();
        return false;       
    }
}

bool ExtPos_Dynamixel::get_currents(std::vector<float>& currents)
{
    // Local Variable useful to convert
    std::vector<int16_t> current_registers;
    
    if(get_CurRegisters(current_registers))
    {
        // Convert
        for(i = 0; i < n_motors; i++)
        {
            currents[i] = register2Current(current_registers[i]); 
        }

        return true;
    }
    else
    {
        // Stop and get false
        return false;
    }
}

bool ExtPos_Dynamixel::set2registers(int32_t registers[])
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_positions[n_motors][POSITION_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        //if(!registerTurns_saturation(registers[i]))
        //    ROS_WARN("Commanded Current are out of limits. Saturating...");

        param_goal_positions[i][0] = DXL_LOBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][1] = DXL_HIBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][2] = DXL_LOBYTE(DXL_HIWORD(registers[i]));
        param_goal_positions[i][3] = DXL_HIBYTE(DXL_HIWORD(registers[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_positions[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setPosition : [ID:%d] [POSITION (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool ExtPos_Dynamixel::set2registers_disable(int32_t registers[])
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_positions[n_motors][POSITION_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        //if(!registerTurns_saturation(registers[i]))
        //    ROS_WARN("Commanded Current are out of limits. Saturating...");

        // Skip if the motor is turned off
        if(!motors_mask[i])
            continue;

        // Extract the word
        param_goal_positions[i][0] = DXL_LOBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][1] = DXL_HIBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][2] = DXL_LOBYTE(DXL_HIWORD(registers[i]));
        param_goal_positions[i][3] = DXL_HIBYTE(DXL_HIWORD(registers[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_positions[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setPosition : [ID:%d] [POSITION (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool ExtPos_Dynamixel::set_turns(float turns[])
{
    // Convert in position register value
    int32_t registers[n_motors];

    // Start Conversion
    i = 0;
    for(i; i < n_motors; i++)
    {
        // Security Saturation on turns values
        if(!turns_saturation(turns[i]))
            ROS_WARN("Commanded Turns are out of limits. Saturating...");

        registers[i] = ((int32_t) (turns[i]*((float) ONE_TURN_REGISTER))) + initial_positions[i];
    }

    return set2registers(registers);
}

bool ExtPos_Dynamixel::set_turns(std::vector<float> turns)
{
    // Convert in position register value
    int32_t registers[n_motors];

    // Start Conversion
    i = 0;
    for(i; i < n_motors; i++)
    {
        // Security Saturation on turns values
        if(!turns_saturation(turns[i]))
            ROS_WARN("Commanded Turns are out of limits. Saturating...");

        registers[i] = ((int32_t) (turns[i]*((float) ONE_TURN_REGISTER))) + initial_positions[i];
    }

    return set2registers(registers);
}

bool ExtPos_Dynamixel::set_turns_disable(std::vector<float> turns)
{
    // Convert in position register value
    int32_t registers[n_motors];

    // Start Conversion
    i = 0;
    for(i; i < n_motors; i++)
    {
        // Manage Disable Torque Request
        if(turns[i] == DISABLE_TORQUE_REQUEST)
        {
            // case 1: Turn off the i-th motor
            if(motors_mask[i])
            {
                // --- Disable Torque & LED --- //
                // LED
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_OFF, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) 
                {
                    ROS_ERROR("Failed to turn off LED for Dynamixel ID %d", i+1);
                    break;
                }

                // Disable Torque
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) 
                {
                    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", i+1);
                    break;
                }

                // Update Motors mask
                motors_mask[i] = false;
            }

            // Case 2: The i-th motor is already turned off
            // else
                // do nothing
        }

        // Manage Enable Torque Request
        else
        {
            // Case 3: The i-th motor turned off
            if(!motors_mask[i])
            {
                // --- Enable Torque & LED --- //
                // LED
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) 
                {
                    ROS_ERROR("Failed to turn on LED for Dynamixel ID %d", i+1);
                    break;
                }

                // Enable Torque
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) 
                {
                    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
                    break;
                }

                // Update motors_mask
                motors_mask[i] = true;
            }
            
            // Case 4: The i-th motor is ready to set turns
            // else
                // do nothing

            // Security Saturation on turns values
            if(!turns_saturation(turns[i]))
                ROS_WARN("Commanded Turns are out of limits. Saturating...");

            registers[i] = ((int32_t) (turns[i]*((float) ONE_TURN_REGISTER))) + initial_positions[i];
        }
    }

    // Avoid writing in the registers when all the motors are turned off
    if(is_allOFF())
    {
        ROS_WARN("All motors are turned OFF.");
        return true;
    }
    else
        return set2registers_disable(registers);
}

bool ExtPos_Dynamixel::set2Zeros()
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_positions[n_motors][POSITION_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_positions[i][0] = DXL_LOBYTE(DXL_LOWORD(initial_positions[i]));
        param_goal_positions[i][1] = DXL_HIBYTE(DXL_LOWORD(initial_positions[i]));
        param_goal_positions[i][2] = DXL_LOBYTE(DXL_HIWORD(initial_positions[i]));
        param_goal_positions[i][3] = DXL_HIBYTE(DXL_HIWORD(initial_positions[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_positions[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setPosition : [ID:%d] [POSITION (register):%d]", i+1, 0); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool ExtPos_Dynamixel::is_allOFF()
{
    // useful variable
    bool output = false;
    
    // Iterative OR to find at least one true
    for(i = 0; i < n_motors; i++)
    {
        output |= motors_mask[i];
    }

    // If all elements of motors_mask are false, then true
    // If motors_mask contains at least 1 true, then false
    return !output;
}