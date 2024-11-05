/**************************************************
 * Useful Functions to use Dynamixel in your Robot*
 **************************************************/

// Dynamixel XM430-W210-R.
// E-Manual: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210

#ifndef DYNAMIXEL_MOTORS_H_
#define DYNAMIXEL_MOTORS_H_

// Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"
// ROS for C++
#include "ros/ros.h"

// Dynamixel Namespace
using namespace dynamixel;

/* DEFINE*/
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
#define PROFILE_ACC_VALUE       10   // Smoother Moves: 10

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
#define ONE_TURN_REGISTER       4095            // uint16_t | Position value that corrisponds to only one turn
#define MAX_TURNS               5.0             // float    | Max Turns
#define MAX_VELOCITY_REGISTER   1023            // uint32_t

// Mapping Torque - Current
// current(torque) = coeff_2*torque^2 + coeff_1*torque + coeff_0
#define COEFF_0 0.1327
#define COEFF_1 0.5753
#define COEFF_2 0.2030

// Functions
int16_t current2Register(float current_value);
float   register2Current(int16_t register_value);
bool    registerCur_saturation(int16_t &register_value);
bool    turns_saturation(float &turn);
float   torque2Current(float current);
int16_t torque2Register(float torque);
float   sign(float x);
int32_t velocity2Register(float velocity_value);

// (Abstract) Dynamixel Class
template <typename T>
class Dynamixel_Motors
{
    // --- Protected Attributes --- //
    protected:
        // Communication Utils    
        PortHandler *portHandler        = PortHandler::getPortHandler(DEVICE_NAME);
        PacketHandler *packetHandler    = PacketHandler::getPacketHandler(BAUDRATE);
        
        // N° of motors
        int n_motors = 0;

        // Error Handling
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Iterator
        int i;  // Increase Comput. Efficiency

    // Methods
    public:
        // --- Constructor --- //
        Dynamixel_Motors();
        
        // --- Deconstructor --- //
        ~Dynamixel_Motors();

        // --- Methods --- //
        virtual bool set2registers(T registers[]){return true;};
        virtual bool set2registers(std::vector<T> registers){return true;};

        // Power Off Functions
        virtual bool set2Zeros(){return true;};
        void powerOFF();                // Turn Off every Dynamixels
};

// --- (Abstract) Dynamixel Class Implementation --- //
// Not present in the source code .cpp for the template syntax

// Constructor 
template <typename T> Dynamixel_Motors<T>::Dynamixel_Motors()
{
    // Open Communication

    // Init of these variables in the declaration
    //uint8_t dxl_error = 0;
    //int dxl_comm_result = COMM_TX_FAIL;

    if(!portHandler->openPort()) 
        ROS_ERROR("Failed to open the port!");

    if(!portHandler->setBaudRate(BAUDRATE))
        ROS_ERROR("Failed to set the baudrate!");
}

// Deconstructor
template <typename T> Dynamixel_Motors<T>::~Dynamixel_Motors()
{
    ROS_WARN("Terminating Dynamixel object...");
    
    // Set Current to Zero in every motors
    if(!set2Zeros())
        ROS_ERROR("Failed to set all the torques to zero.");
    
    // Turn Off every motors
    powerOFF();
    
    ROS_WARN("Dynamixel Object terminated.");
}

// Security Power OFF
template <typename T> void Dynamixel_Motors<T>::powerOFF()
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    // Turn Off LED and Disable Torque
    i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_OFF, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn off LED for Dynamixel ID %d", i+1);
            //break;
        }

        // Disable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to disable torque for Dynamixel ID %d", i+1);
            //break;
        }
    }
}

#endif /* DYNAMIXEL_MOTORS_H_ */