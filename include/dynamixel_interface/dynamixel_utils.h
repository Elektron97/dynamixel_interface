/**************************************************
 * Register-level constants and conversion utils  *
 * shared by every Dynamixel command/feedback mode*
 **************************************************/

// Dynamixel XM430-W210-R.
// E-Manual: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210

#ifndef DYNAMIXEL_UTILS_H_
#define DYNAMIXEL_UTILS_H_

// Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"
// ROS for C++
#include "ros/ros.h"

// Dynamixel Namespace
using namespace dynamixel;

/* DEFINE */
// Addresses
#define ADDR_TORQUE_ENABLE      64
#define ADDR_LED                65
#define ADDR_OP_MODE            11
// Goal
#define ADDR_GOAL_POSITION      116
#define ADDR_GOAL_VELOCITY      104
#define ADDR_GOAL_CURRENT       102
// Present
#define ADDR_PRESENT_POSITION   132
#define ADDR_PRESENT_CURRENT    126
// Present-current, -velocity and -position are laid out contiguously in the
// control table (126-135), so a single sync read spanning all of them costs
// one bus transaction instead of two - worth it since the shared serial bus,
// not the CPU, is what actually caps the achievable feedback rate.
#define FEEDBACK_START_ADDR     ADDR_PRESENT_CURRENT
#define FEEDBACK_BYTE_LENGTH    ((ADDR_PRESENT_POSITION + POSITION_BYTE) - ADDR_PRESENT_CURRENT)
// Profile
#define ADDR_PROFILE_VEL        112     // 4 bytes
#define ADDR_PROFILE_ACC        108     // 4 bytes

// Value
#define LED_ON                  1
#define LED_OFF                 0
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define CURRENT_MODE            0
#define VELOCITY_MODE           1
#define EXTENDED_POSITION_MODE  4
#define CURRENT_POSITION_MODE   5
#define PROFILE_VEL_VALUE       100   // Smoother Moves: 100
#define PROFILE_ACC_VALUE       10    // Smoother Moves: 10

// Data Length
#define POSITION_BYTE           4
#define VELOCITY_BYTE           4
#define CURRENT_BYTE            2

#define PROTOCOL_VERSION        2.0             // Default Protocol version of DYNAMIXEL X series.

// Hardware Parameters
#define BAUDRATE                115200
#define DEVICE_NAME             "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

// Limit
#define MAX_CURRENT             3.209           // [A] | Max current permitted in the motors.
#define MAX_VELOCITY            24.5324         // [rad/s]

#define MAX_CURRENT_REGISTER    1193            // uint16_t | Max value in the current Register. Corresponds to MAX_CURRENT.
#define ONE_TURN_REGISTER       4096            // uint16_t | Register counts per one full revolution (XM430-W210 encoder resolution).
#define MAX_TURNS               3.0             // float    | Default Max Turns (overridable at runtime, see ~max_turns)
#define MAX_VELOCITY_REGISTER   1023            // uint32_t

// Mapping Torque - Current
// current(|torque|) = coeff_2*|torque|^2 + coeff_1*|torque| + coeff_0 (sign re-applied afterwards)
#define COEFF_0 0.1327
#define COEFF_1 0.5753
#define COEFF_2 0.2030

// --- Conversion / Saturation Functions --- //
int16_t current2Register(float current_value);
float   register2Current(int16_t register_value);
bool    registerCur_saturation(int16_t &register_value);

float   register2Turns(int32_t register_value, int32_t initial_position);
bool    turns_saturation(float &turn, float max_turns = MAX_TURNS);

float   sign(float x);

float   torque2Current(float torque);
float   current2Torque(float current);
int16_t torque2Register(float torque);
float   register2Torque(int16_t register_value);

int32_t velocity2Register(float velocity_value);

#endif /* DYNAMIXEL_UTILS_H_ */
