/****************************************
 * Source code for conversion utilities *
 ****************************************/
#include "dynamixel_interface/dynamixel_utils.h"
#include <cmath>

int16_t current2Register(float current_value)
{
    return (int16_t)((current_value / MAX_CURRENT) * MAX_CURRENT_REGISTER);
}

float register2Current(int16_t register_value)
{
    return MAX_CURRENT * ((float) register_value / MAX_CURRENT_REGISTER);
}

bool registerCur_saturation(int16_t &register_value)
{
    // No need of sign function, because is only positive values
    if(std::abs(register_value) > MAX_CURRENT_REGISTER)
    {
        register_value = ((int16_t) sign(register_value)) * MAX_CURRENT_REGISTER;
        return false;
    }
    else
    {
        return true;
    }
}

float register2Turns(int32_t register_value, int32_t initial_position)
{
    return ((float) (register_value - initial_position) / (float) ONE_TURN_REGISTER);
}

bool turns_saturation(float &turn, float max_turns)
{
    if(std::fabs(turn) > max_turns)
    {
        turn = sign(turn) * max_turns;
        return false;
    }
    else
        return true;
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
    // The quadratic fit was characterized on |torque| -> |current|; reapply the
    // sign explicitly so this stays consistent with current2Torque's inverse below
    // (a sign-blind version would return a positive current for any negative torque).
    float current_abs = COEFF_2 * torque * torque + COEFF_1 * std::fabs(torque) + COEFF_0;
    return sign(torque) * current_abs;
}

float current2Torque(float current)
{
    // Inverse Solution of:
    // COEFF_2*torque^2 + COEFF_1*torque + COEFF_0 = |current|
    float torque_abs = -(COEFF_1 - std::sqrt(COEFF_1*COEFF_1 - 4*COEFF_0*COEFF_2 + 4*COEFF_2*std::fabs(current)))/(2*COEFF_2);

    // Sign correction
    return sign(current)*torque_abs; // return in [Nm]
}

int16_t torque2Register(float torque)
{
    return current2Register(torque2Current(torque));
}

float register2Torque(int16_t register_value)
{
    return current2Torque(register2Current(register_value));
}

int32_t velocity2Register(float velocity_value)
{
    return (int32_t)((velocity_value / MAX_VELOCITY) * MAX_VELOCITY_REGISTER);
}
