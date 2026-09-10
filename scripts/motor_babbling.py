#!/usr/bin/env python3
"""
Motor babbling: repeatedly publishes a fresh random turns vector (independent
uniform draw per motor) on /dynamixels/cmd_turns, holding each draw for
~hold_s seconds, for a total of ~duration_s seconds. motor_io must be running
in "turns" or "current_position" command_mode.
"""
import random

import rospy
from std_msgs.msg import Float32MultiArray

N_MOTORS = 7
LOW = -1.0
HIGH = 0.0
HOLD_S = 2.0
DURATION_S = 30.0


def main():
    rospy.init_node("motor_babbling")
    low = rospy.get_param("~low", LOW)
    high = rospy.get_param("~high", HIGH)
    hold_s = rospy.get_param("~hold_s", HOLD_S)
    duration_s = rospy.get_param("~duration_s", DURATION_S)

    pub = rospy.Publisher("/dynamixels/cmd_turns", Float32MultiArray, queue_size=10)
    rospy.sleep(0.5)  # let publisher connect

    n_draws = int(duration_s / hold_s)
    for i in range(n_draws):
        if rospy.is_shutdown():
            break
        values = [random.uniform(low, high) for _ in range(N_MOTORS)]
        rospy.loginfo("babbling draw %d/%d: %s", i + 1, n_draws, ["%.3f" % v for v in values])
        pub.publish(Float32MultiArray(data=values))
        rospy.sleep(hold_s)

    pub.publish(Float32MultiArray(data=[0.0] * N_MOTORS))


if __name__ == "__main__":
    main()
