/****************************
 * Motor Input/Output Node  *
 ****************************/
// --- Includes --- //
#include "dynamixel_interface/ros_utils.h"

// --- Main --- //
int main(int argc, char** argv)
{
    // --- Init ROS Node --- //
    ros::init(argc, argv, "motor_io");

    // Init Motors
    Ros_Dynamixel_Node node_obj;

    // --- Main Loop --- //
    ros::spin();

    // --- End of Program --- //
    ROS_INFO("End of Main");
    return 0;
}