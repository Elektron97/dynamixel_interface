/****************************************
 * Source code for Dynamixel Library    *
 ****************************************/
// Include
#include "dynamixel_interface/dynamixel_motors.h"

// Implementation in the header due to template synatx
/**************************************************************************/
// --- Functions --- //
int16_t current2Register(float current_value)
{
    return MAX_CURRENT_REGISTER*((int16_t) (current_value/MAX_CURRENT));
}

float register2Current(int16_t register_value)
{
    return MAX_CURRENT*(((float) register_value)/ ((float) MAX_CURRENT_REGISTER));
}

bool registerCur_saturation(int16_t &register_value) 
{
    // No need of sign function, because is only positive values
    if(abs(register_value) > MAX_CURRENT_REGISTER)
    {
        register_value = ((int16_t) sign(register_value))*MAX_CURRENT_REGISTER;
        return false;
    }
    else
    {
        return true;
    }
}

bool turns_saturation(float &turn) 
{
    // No need of sign function, because is only positive values
    if(turn > MAX_TURNS)
    {
        turn = MAX_TURNS;
        return false;
    }
    // No negative turns
    else if (turn < 0)
    {
        turn = 0.0;
        return false;
    }
    else
    {
        return true;
    }
}

float sign(float x)
{
    /*SIGN FUNCTION:*/
    if(x > 0)
        return 1.0;
    if(x < 0)
        return -1.0;
    else
        return 0.0;
}

float torque2Current(float torque)
{
    return COEFF_2*torque*torque + COEFF_1*torque + COEFF_0;
}

int16_t torque2Register(float torque)
{
    return current2Register(torque2Current(torque));
}

int32_t velocity2Register(float velocity_value)
{
    return MAX_VELOCITY_REGISTER*((int32_t) (velocity_value/MAX_VELOCITY));
}