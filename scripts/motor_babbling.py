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
PROX = (1, 4, 5)  # motor indices (ID-1) of channels 2,5,6: base -> cs1
PROX_LOW = -1.0
DIST_LOW = -2.0
HIGH = 0.0
HOLD_S = 2.0
DURATION_S = 30.0


def main():
    rospy.init_node("motor_babbling")
    prox_low = rospy.get_param("~prox_low", PROX_LOW)
    dist_low = rospy.get_param("~dist_low", DIST_LOW)
    lows = [prox_low if i in PROX else dist_low for i in range(N_MOTORS)]
    high = rospy.get_param("~high", HIGH)
    hold_s = rospy.get_param("~hold_s", HOLD_S)
    duration_s = rospy.get_param("~duration_s", DURATION_S)

    pub = rospy.Publisher("/dynamixels/cmd_turns", Float32MultiArray, queue_size=10)
    rospy.sleep(0.5)  # let publisher connect

    n_draws = int(duration_s / hold_s)
    for i in range(n_draws):
        if rospy.is_shutdown():
            break
        values = [random.uniform(lo, high) for lo in lows]
        rospy.loginfo("babbling draw %d/%d: %s", i + 1, n_draws, ["%.3f" % v for v in values])
        pub.publish(Float32MultiArray(data=values))
        rospy.sleep(hold_s)

    pub.publish(Float32MultiArray(data=[0.0] * N_MOTORS))


if __name__ == "__main__":
    main()
