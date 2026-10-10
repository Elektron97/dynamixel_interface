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
// Default feedback publish rate, overridable via the "~node_frequency" ROS
// param. Historically capped near this value by a bug in the feedback path
// (fixed - see history in dynamixel_interface.cpp); ~node_frequency lets a
// launch file now go higher if the bus/motor count can keep up.
#define NODE_FREQUENCY  30.0    // [Hz]
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
    // turns_pub is advertised only if the "~publish_turns" param is true
    // (default false): read_turns feedback was found far noisier than
    // read_currents in recent experiments (see publish_feedback()), but
    // some nodes (e.g. eleanor_babble) need it.
    ros::Publisher turns_pub;

    // Service
    ros::ServiceServer torque_srv = node_handle.advertiseService(torque_srv_name, &Ros_Dynamixel_Node::torque_server, this);

    // Timer: constructed in the constructor body (not here as a default
    // member initializer), since its period now comes from the runtime
    // "~node_frequency" param rather than the NODE_FREQUENCY compile-time
    // default.
    ros::Timer timer_obj;

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
        // DynamixelInterface::get_feedback, since currents share the same
        // sync read as turns) but only publishes currents for now -
        // turns_pub is commented out, not removed (see dynamixel_node.cpp).
        void publish_feedback();

        // Main Loop
        void main_loop(const ros::TimerEvent& event);
};


#endif /* DYNAMIXEL_NODE_H_ */
