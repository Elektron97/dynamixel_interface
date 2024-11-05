#ifndef EXTPOS_DYNAMIXEL_H_
#define EXTPOS_DYNAMIXEL_H_

#include "ros_dynamixel_pkg/dynamixel_motors.h"

#define DISABLE_TORQUE_REQUEST -1

// Extended Position Dynamixel with Current Feedback
class ExtPos_Dynamixel: public Dynamixel_Motors<int32_t>
{
    // Init with POSITION address
    GroupSyncWrite  motors_syncWrite    = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, POSITION_BYTE);
    GroupSyncRead   position_syncRead   = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, POSITION_BYTE);
    GroupSyncRead   current_syncRead    = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, CURRENT_BYTE);

    // Initial Positions
    std::vector<int32_t> initial_positions;

    // Motors' Mask
    std::vector<bool> motors_mask;

    /************************************************  
    *   These methods are private because           *
    *   there are no saturation on them.            *
    *   The user has to use only the set_turns()    *
    *   method.                                     *
    *************************************************/
    // Low Level Set: Register
    bool set2registers(int32_t registers[]);
    bool set2registers_disable(int32_t registers[]);

    public:
        // --- Constructor --- //
        ExtPos_Dynamixel(int n_motors);

        // --- Methods --- //
        bool set_turns(float turns[]);
        bool set_turns(std::vector<float> turns);

        // set_turns w/ disable_request
        bool set_turns_disable(std::vector<float> turns);

        // Low Level Get: Register
        bool get_PosRegisters(std::vector<int32_t>& positions);
        bool get_CurRegisters(std::vector<int16_t>& currents);

        // Mid Level Get: Current
        bool get_currents(std::vector<float>& currents);

        // Power Off Functions
        bool set2Zeros();

        // Check if all the motors are turned off
        bool is_allOFF();
};


#endif /* EXTPOS_DYNAMIXEL_H_ */