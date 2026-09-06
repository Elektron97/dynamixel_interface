/****************************************************
 * Motor Input/Output Node                          *
 * Wraps DynamixelInterface for either command mode *
 ****************************************************/
#ifndef DYNAMIXEL_NODE_H_
#define DYNAMIXEL_NODE_H_

// --- Includes --- //
#include "dynamixel_interface/dynamixel_interface.h"

// msgs
#include "std_msgs/Float32MultiArray.h"

// srv
#include "std_srvs/SetBool.h"

// --- Define --- //
#define NODE_FREQUENCY  30.0    // [Hz] Max publish rate is ~31 Hz
#define QUEUE_SIZE      10
#define N_MOTORS        7

// --- Namespace --- //
using namespace std;        // std io

// --- Global Variables --- //
const string topic_tag          = "/dynamixels";
const string cmd_currents_name  = "/cmd_currents";
const string cmd_torques_name   = "/cmd_torques";
const string cmd_turns_name     = "/cmd_turns";
const string read_currents_name = "/read_currents";
const string read_turns_name    = "/read_turns";
const string torque_srv_name    = "/switch_torque";

class Ros_Dynamixel_Node
{
    //--- Private Attributes ---//
    // - Ros objects - //
    ros::NodeHandle node_handle;

    // Command Mode, resolved once from the "~command_mode" private param
    // ("current" or "turns") before dyna_obj is constructed.
    CommandMode mode;

    // - Dynamixel Object - //
    DynamixelInterface dyna_obj;

    // Sub & Pub objects. cmd_sub's topic name follows `mode` (set in the
    // constructor body), so only the topic matching the configured mode is
    // ever subscribed. cmd_torques_sub is additionally subscribed only when
    // mode == CURRENT (an alternate, Nm-unit input onto the same current
    // command path as cmd_currents - publish to one or the other, not both).
    ros::Subscriber cmd_sub;
    ros::Subscriber cmd_torques_sub;
    ros::Publisher currents_pub = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + read_currents_name, QUEUE_SIZE);
    ros::Publisher turns_pub    = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + read_turns_name, QUEUE_SIZE);

    // Service
    ros::ServiceServer torque_srv = node_handle.advertiseService(torque_srv_name, &Ros_Dynamixel_Node::torque_server, this);

    // Timer
    ros::Timer timer_obj = node_handle.createTimer(ros::Duration(1/NODE_FREQUENCY), &Ros_Dynamixel_Node::main_loop, this);

    // Useful Variables
    std_msgs::Float32MultiArray motor_turns;
    std_msgs::Float32MultiArray motor_currents;

    // Callbacks
    void cmd_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void cmd_torques_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    bool torque_server(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    public:
        // Constructor
        Ros_Dynamixel_Node();

        // Deconstructor
        ~Ros_Dynamixel_Node();

        // Publisher: reads turns+currents together (one bus transaction via
        // DynamixelInterface::get_feedback) and publishes both.
        void publish_feedback();

        // Main Loop
        void main_loop(const ros::TimerEvent& event);
};


#endif /* DYNAMIXEL_NODE_H_ */
