#!/usr/bin/env python3
"""
Per-motor turns test sequence: for each motor in turn, jump 0->target turn, hold 10s,
ramp target->0 over 10s, then move to the next motor. Publishes on /dynamixels/cmd_turns
(motor_io must be running in "turns" or "current_position" command_mode).
"""
import rospy
from std_msgs.msg import Float32MultiArray

N_MOTORS = 7
IDLE_S = 10.0
HOLD_S = 10.0
RAMP_S = 10.0
RAMP_HZ = 20
TURNS = -1.0


def main():
    rospy.init_node("turns_sequence_test")
    idle_s = rospy.get_param("~idle_s", IDLE_S)
    hold_s = rospy.get_param("~hold_s", HOLD_S)
    ramp_s = rospy.get_param("~ramp_s", RAMP_S)
    ramp_hz = rospy.get_param("~ramp_hz", RAMP_HZ)
    turns = rospy.get_param("~turns", TURNS)
    pub = rospy.Publisher("/dynamixels/cmd_turns", Float32MultiArray, queue_size=10)
    rospy.sleep(0.5)  # let publisher connect

    def publish(values):
        pub.publish(Float32MultiArray(data=values))

    values = [0.0] * N_MOTORS
    rate = rospy.Rate(ramp_hz)

    rospy.loginfo("idle %.0fs before starting sequence", idle_s)
    publish(values)
    rospy.sleep(idle_s)

    for motor in range(N_MOTORS):
        if rospy.is_shutdown():
            break
        rospy.loginfo("motor %d: 0 -> %.3f turns, hold %.0fs", motor + 1, turns, hold_s)
        values[motor] = turns
        publish(values)
        rospy.sleep(hold_s)

        rospy.loginfo("motor %d: ramping %.3f -> 0 over %.0fs", motor + 1, turns, ramp_s)
        steps = int(ramp_s * ramp_hz)
        for s in range(steps, -1, -1):
            if rospy.is_shutdown():
                break
            values[motor] = turns * (s / float(steps))
            publish(values)
            rate.sleep()
        values[motor] = 0.0
        publish(values)

    publish([0.0] * N_MOTORS)


if __name__ == "__main__":
    main()
