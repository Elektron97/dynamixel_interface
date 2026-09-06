/****************************************
 * Source code for ROS Node Library     *
 ****************************************/
#include "dynamixel_interface/dynamixel_node.h"
#include <algorithm>

namespace
{
    // Reads the "~command_mode" private param ("current"/"turns"/
    // "current_position", default "current") before DynamixelInterface is
    // constructed. Mode is launch-time only: switching it requires restarting
    // the node, matching how the hardware's operating-mode register actually
    // works (torque must be disabled to reprogram it), so there is no runtime
    // mode-switch service.
    CommandMode resolveCommandMode()
    {
        ros::NodeHandle private_nh("~");
        string mode_str;
        private_nh.param<string>("command_mode", mode_str, "current");

        if(mode_str == "turns")
            return CommandMode::TURNS;
        if(mode_str == "current_position")
            return CommandMode::CURRENT_POSITION;

        if(mode_str != "current")
            ROS_WARN("Unknown command_mode '%s', defaulting to 'current'.", mode_str.c_str());

        return CommandMode::CURRENT;
    }

    // Reads the "~current_limit" private param [A], only meaningful for
    // CommandMode::CURRENT_POSITION, where it's latched once as the position
    // controller's torque ceiling. Defaults to MAX_CURRENT (i.e. no effective
    // limiting) if unset - warns loudly in that case, since an unset limit
    // silently defeats the point of choosing this mode.
    float resolveCurrentLimit(CommandMode mode)
    {
        ros::NodeHandle private_nh("~");
        bool has_param = private_nh.hasParam("current_limit");

        double limit;
        private_nh.param<double>("current_limit", limit, (double) MAX_CURRENT);

        if(mode == CommandMode::CURRENT_POSITION && !has_param)
            ROS_WARN("command_mode is 'current_position' but ~current_limit was not set - "
                      "defaulting to MAX_CURRENT (%.3f A), i.e. no effective torque limiting. "
                      "Set ~current_limit [A] for compliant position control.", MAX_CURRENT);

        if(limit <= 0.0 || limit > MAX_CURRENT)
        {
            ROS_WARN("~current_limit (%.3f A) is out of range (0, %.3f]; clamping.", limit, MAX_CURRENT);
            limit = std::min(std::max(limit, 0.0), (double) MAX_CURRENT);
        }

        return (float) limit;
    }

    const char* modeName(CommandMode mode)
    {
        switch(mode)
        {
            case CommandMode::CURRENT:          return "current";
            case CommandMode::TURNS:             return "turns";
            case CommandMode::CURRENT_POSITION:  return "current_position";
        }
        return "unknown";
    }
}

// --- ROS DYNAMIXEL NODE CLASS --- //
Ros_Dynamixel_Node::Ros_Dynamixel_Node()
    : mode(resolveCommandMode())
    , dyna_obj(N_MOTORS, mode, resolveCurrentLimit(mode))
{
    // Command Subscriber: topic name follows the configured mode (both TURNS
    // and CURRENT_POSITION are commanded by turn count), so the node
    // structurally only ever listens on the one that matches it.
    const string& cmd_topic = (mode == CommandMode::CURRENT) ? cmd_currents_name : cmd_turns_name;
    cmd_sub = node_handle.subscribe(topic_tag + cmd_topic, QUEUE_SIZE, &Ros_Dynamixel_Node::cmd_callBack, this);

    // In CURRENT mode, also accept torque [Nm] commands on a second topic,
    // converted via the COEFF_*-fit torque2Current/torque2Register. Both
    // topics drive the same current command path - publish to one or the
    // other, not both at once.
    if(mode == CommandMode::CURRENT)
        cmd_torques_sub = node_handle.subscribe(topic_tag + cmd_torques_name, QUEUE_SIZE, &Ros_Dynamixel_Node::cmd_torques_callBack, this);

    // Init FloatMultiArray
    motor_turns.data    = vector<float>(N_MOTORS);
    motor_currents.data = vector<float>(N_MOTORS);

    if(!dyna_obj.allMotorsReady())
        ROS_ERROR("Not all Dynamixel motors initialized correctly. Check wiring/IDs/power.");

    ROS_INFO("Dynamixel node started in '%s' command mode, subscribing on '%s'.",
              modeName(mode), (topic_tag + cmd_topic).c_str());
}

Ros_Dynamixel_Node::~Ros_Dynamixel_Node()
{
    // Deconstructor
    ROS_INFO("Shutting down Dynamixel node.");
}

void Ros_Dynamixel_Node::cmd_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if((int) msg->data.size() != N_MOTORS)
    {
        ROS_ERROR("Command message is incorrect. Expected %d motors, got %zu.", N_MOTORS, msg->data.size());
        return;
    }

    if(dyna_obj.set_command(msg->data))
        ROS_DEBUG("Command written correctly on Dynamixels.");
    else
        ROS_ERROR("Command written incorrectly on Dynamixels.");
}

void Ros_Dynamixel_Node::cmd_torques_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if((int) msg->data.size() != N_MOTORS)
    {
        ROS_ERROR("Torque command message is incorrect. Expected %d motors, got %zu.", N_MOTORS, msg->data.size());
        return;
    }

    if(dyna_obj.set_torques(msg->data))
        ROS_DEBUG("Torque command written correctly on Dynamixels.");
    else
        ROS_ERROR("Torque command written incorrectly on Dynamixels.");
}

bool Ros_Dynamixel_Node::torque_server(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    // Enable or Disable Torque
    bool ok = req.data ? dyna_obj.enableTorque() : dyna_obj.disableTorque();

    // Response reflects what actually happened on the hardware
    res.success = ok;
    res.message = ok ? "Torque switched successfully." : "Failed to switch torque on one or more motors. Check logs.";

    return true;
}

void Ros_Dynamixel_Node::publish_feedback()
{
    // Reads directly into the persistent message members (already sized to
    // N_MOTORS) rather than a fresh local vector, and get_feedback() does
    // both turns and currents in one bus transaction - so a steady-state
    // control tick performs zero heap allocation here.
    if(!dyna_obj.get_feedback(motor_turns.data, motor_currents.data))
    {
        ROS_ERROR("Failed to read motor feedback.");
        return;
    }

    turns_pub.publish(motor_turns);
    currents_pub.publish(motor_currents);
}

void Ros_Dynamixel_Node::main_loop(const ros::TimerEvent& event)
{
    // Feedback is mode-independent: always publish both.
    publish_feedback();
}
