/****************************************
 * Source code for ROS Node Library     *
 ****************************************/
#include "dynamixel_interface/ros_utils.h"

// --- ROS DYNAMIXEL NODE CLASS --- //
Ros_Dynamixel_Node::Ros_Dynamixel_Node()
{
    // Init FloatMultiArray
    motor_currents.data = vector<float>(N_MOTORS);
}

Ros_Dynamixel_Node::~Ros_Dynamixel_Node()
{
    // Deconstructor
    ROS_INFO("Turning OFF all the motors."); 
}

void Ros_Dynamixel_Node::turns_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("Turns command received.");

    // Verify n of motors
    if(msg->data.size() == N_MOTORS)
    {
        //Extract array of torques
        if(dyna_obj.set_turns_disable(msg->data))
            ROS_INFO("Turns command written correctly on Dynamixels.");
        else
        {
            ROS_ERROR("Turns command written uncorrectly on Dynamixels.");
            return;
        }
    }
    else
    {
        ROS_ERROR("/cmd_turns msg is uncorrect. Wrong number of motors.");
        return;
    }
}

void Ros_Dynamixel_Node::publish_currents()
{
    vector<float> currents(N_MOTORS);
    if(!dyna_obj.get_currents(currents))
    {
        ROS_ERROR("Failed to read currents.");
        return;
    }
    else
    {
        motor_currents.data = currents;
        // Publish only if the reading is ok
        current_pub.publish(motor_currents);
    }
}

void Ros_Dynamixel_Node::main_loop(const ros::TimerEvent& event)
{
    // Publish /read_currents
    publish_currents();
}