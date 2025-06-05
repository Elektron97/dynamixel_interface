#include "ros_dynamixel_pkg/current_dynamixel.h"

// --- Current Dynamixel Class --- //
Current_Dynamixel::Current_Dynamixel(int n_dyna)
{
    // Init n_motors
    n_motors = n_dyna;

    // Error Handling
    // Init done by Super Class Constructor
    //dxl_error = 0;
    //dxl_comm_result = COMM_TX_FAIL;

    // Turn On LED, Current Mode and Enable Torque
    i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn on LED for Dynamixel ID %d", i+1);
            break;
        }

        // Current Drive Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_OP_MODE, CURRENT_MODE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set Current Mode for Dynamixel ID %d", i+1);
            break;
        }

        // Enable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Set Current to Zero in every motors
    if(!set2Zeros())
        ROS_ERROR("Failed to set all the torques to zero.");
}

bool Current_Dynamixel::set2Zeros()
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_currents[i][0] = DXL_LOBYTE(0);
        param_goal_currents[i][1] = DXL_HIBYTE(0);

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_currents[i]); // da sistemare
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
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, 0); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Current_Dynamixel::set2registers(int16_t registers[])
{
    // Assert: check that dim(currents) = n_motors
    //assert(sizeof(currents)/sizeof(int16_t) == n_motors);

    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        if(!registerCur_saturation(registers[i]))
            ROS_WARN("Commanded Current are out of limits. Saturating...");


        param_goal_currents[i][0] = DXL_LOBYTE(registers[i]);
        param_goal_currents[i][1] = DXL_HIBYTE(registers[i]);

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_currents[i]); // da sistemare
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
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Current_Dynamixel::set2registers(std::vector<int16_t> registers)
{
    // Assert: check that dim(currents) = n_motors
    //assert(sizeof(currents)/sizeof(int16_t) == n_motors);

    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        if(!registerCur_saturation(registers[i]))
            ROS_WARN("Commanded Current are out of limits. Saturating...");

        param_goal_currents[i][0] = DXL_LOBYTE(registers[i]);
        param_goal_currents[i][1] = DXL_HIBYTE(registers[i]);

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_currents[i]); // da sistemare
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
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Current_Dynamixel::get_CurRegisters(std::vector<int16_t>& currents)
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

bool Current_Dynamixel::set_currents(float currents[])
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = current2Register(currents[i]);
    }

    return set2registers(registers);
}

// Overloading
bool Current_Dynamixel::set_currents(std::vector<float> currents)
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = current2Register(currents[i]);
    }

    return set2registers(registers);
}

bool Current_Dynamixel::set_torques(float torques[])
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = torque2Register(torques[i]);
    }

    return set2registers(registers);
}

bool Current_Dynamixel::get_currents(std::vector<float>& currents)
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

// Overloading
bool Current_Dynamixel::set_torques(std::vector<float> torques)
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = torque2Register(torques[i]);
    }

    return set2registers(registers);
}
