#ifndef CURRENT_DYNAMIXEL_H_
#define CURRENT_DYNAMIXEL_H_

#include "ros_dynamixel_pkg/dynamixel_motors.h"

// Current Dynamixel
class Current_Dynamixel: public Dynamixel_Motors<int16_t>
{
    // Init with CURRENT address
    GroupSyncWrite motors_syncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, CURRENT_BYTE);
    GroupSyncRead current_syncRead  = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, CURRENT_BYTE);

    public:
        // --- Constructor --- //
        Current_Dynamixel(int n_dyna);

        // --- Methods --- //
        // Low Level Set/Get: Register
        bool set2registers(int16_t registers[]);
        bool set2registers(std::vector<int16_t> registers);         // Overwrite (vector<T>)
        bool get_CurRegisters(std::vector<int16_t>& currents);
        
        // Mid Level Set/Get: Current
        bool set_currents(float currents[]);
        bool set_currents(std::vector<float> currents);             // Overwrite (vector<T>)
        bool get_currents(std::vector<float>& currents);

        // High Level Set: Torque   
        bool set_torques(float torques[]);
        bool set_torques(std::vector<float> torques);               // Overwrite (vector<T>)

        // Power Off Functions
        bool set2Zeros();
};

#endif /* CURRENT_DYNAMIXEL_H_ */