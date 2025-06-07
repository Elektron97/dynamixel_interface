/****************************
 * Motor Input/Output Node  *
 ****************************/
#ifndef CURRENT_NODE_H_
#define CURRENT_NODE_H_

/****************************
 * Motor Input/Output Node  *
 ****************************/
// --- Includes --- //
#include "dynamixel_interface/current_dynamixel.h"

// msgs
#include "std_msgs/Float32MultiArray.h"

// srv
#include "std_srvs/SetBool.h"

// --- Define --- //
#define NODE_FREQUENCY  10.0    // [Hz] Max publish rate is ~31 Hz
#define QUEUE_SIZE      10
#define N_MOTORS        7

// --- Namespace --- //
using namespace std;        // std io

// --- Global Variables --- //
const string topic_tag = "/dynamixels";
const string torque_topic_name = "/cmd_currents";
const string current_topic_name = "/read_currents";
const string turns_topic_name = "/read_turns";
const string torque_srv_name = "/switch_torque";

// ---  Function Signatures --- //

class Ros_Dynamixel_Node
{
    //--- Private Attributes ---//
    // - Ros objects - //
    ros::NodeHandle node_handle;
    // Sub & Pub objects
    ros::Subscriber torque_sub   = node_handle.subscribe(topic_tag + torque_topic_name, QUEUE_SIZE, &Ros_Dynamixel_Node::torque_callBack, this);
    // ros::Publisher current_pub  = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + current_topic_name, QUEUE_SIZE);
    ros::Publisher turns_pub  = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + turns_topic_name, QUEUE_SIZE);

    // Service
    ros::ServiceServer torque_srv = node_handle.advertiseService(torque_srv_name, &Ros_Dynamixel_Node::torque_server, this); 

    // Timer
    ros::Timer timer_obj        = node_handle.createTimer(ros::Duration(1/NODE_FREQUENCY), &Ros_Dynamixel_Node::main_loop, this);

    // - Dynamixel Object - //
    Current_Dynamixel dyna_obj   = Current_Dynamixel(N_MOTORS);

    // Useful Variables
    // std_msgs::Float32MultiArray motor_currents;
    std_msgs::Float32MultiArray motor_turns;

    // Callbacks
    void torque_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    bool torque_server(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    
    public:
        // Constructor
        Ros_Dynamixel_Node();

        // Deconstructor
        ~Ros_Dynamixel_Node();

        // Publisher
        // void publish_currents();
        void publish_turns();

        // Main Loop
        void main_loop(const ros::TimerEvent& event);
};


#endif /* CURRENT_NODE_H_ */