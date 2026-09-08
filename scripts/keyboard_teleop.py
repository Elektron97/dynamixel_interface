#!/usr/bin/env python
"""
Keyboard teleop node for dynamixel_interface.

Select a motor with the number keys 1-7, then nudge its commanded value up/down
with w/s (or +/-). Publishes onto whichever command topic motor_io is listening
on:

    current                  -> /dynamixels/cmd_currents [A]
    turns / current_position -> /dynamixels/cmd_turns [turns]

If ~command_mode isn't set explicitly, it's auto-detected at startup by asking
the ROS master which of the two command topics already has a subscriber (i.e.
which one motor_io picked). That only works if motor_io is already running;
otherwise this falls back to "current" and logs a warning. Set ~command_mode
explicitly to skip auto-detection (e.g. if you're starting this before
motor_io, or motor_io itself hasn't come up yet).
"""
from __future__ import print_function

import sys
import select
import termios
import tty

import rospy
import rosgraph
from std_msgs.msg import Float32MultiArray

N_MOTORS = 7
CMD_CURRENTS_TOPIC = "/dynamixels/cmd_currents"
CMD_TURNS_TOPIC = "/dynamixels/cmd_turns"


def detect_command_mode():
    """Infer "current" vs "turns" from which command topic already has a
    subscriber (motor_io), by querying the ROS master's system state.
    Can't distinguish "turns" from "current_position" (both subscribe to the
    same topic) - that's fine, they publish the same way. Returns None if
    neither topic has a subscriber yet (motor_io not up, or not reachable)."""
    try:
        master = rosgraph.Master("/keyboard_teleop")
        _publishers, subscribers, _services = master.getSystemState()
    except Exception as exc:
        rospy.logwarn("keyboard_teleop: couldn't query ROS master for auto-detection: %s", exc)
        return None

    subscribed_topics = {topic for topic, _nodes in subscribers}
    if CMD_TURNS_TOPIC in subscribed_topics:
        return "turns"
    if CMD_CURRENTS_TOPIC in subscribed_topics:
        return "current"
    return None

HELP = """
Dynamixel keyboard teleop
--------------------------
1-7         select motor (currently: {sel})
w / +       increase selected motor's commanded value by step
s / -       decrease selected motor's commanded value by step
r           zero the selected motor
space       zero ALL motors
h           show this help
q / CTRL-C  quit (zeroes all motors and exits)
--------------------------
"""


class KeyboardTeleop(object):
    def __init__(self):
        if rospy.has_param("~command_mode"):
            command_mode = rospy.get_param("~command_mode")
            if command_mode not in ("current", "turns", "current_position"):
                rospy.logwarn(
                    "keyboard_teleop: unknown ~command_mode '%s', ignoring and auto-detecting instead",
                    command_mode,
                )
                command_mode = detect_command_mode()
        else:
            command_mode = detect_command_mode()
            if command_mode is not None:
                rospy.loginfo("keyboard_teleop: auto-detected command_mode=%s from motor_io's subscriptions", command_mode)

        if command_mode is None:
            rospy.logwarn(
                "keyboard_teleop: could not auto-detect command_mode (is motor_io running?); "
                "defaulting to 'current'. Set ~command_mode explicitly to silence this."
            )
            command_mode = "current"
        self.command_mode = command_mode

        # current mode publishes Amps on cmd_currents; turns / current_position
        # both publish turn counts on cmd_turns (same command shape, see
        # DynamixelInterface::set_command).
        if self.command_mode == "current":
            self.unit = "A"
            self.step = rospy.get_param("~current_step", 0.05)
            self.limit = rospy.get_param("~current_limit", 1.0)
            topic = "/dynamixels/cmd_currents"
        else:
            self.unit = "turns"
            self.step = rospy.get_param("~turns_step", 0.05)
            self.limit = rospy.get_param("~turns_limit", 3.0)
            topic = "/dynamixels/cmd_turns"

        self.pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        self.values = [0.0] * N_MOTORS
        self.selected = 0

        rospy.loginfo(
            "keyboard_teleop: command_mode=%s, publishing [%s] on %s (step=%.3f, limit=%.3f)",
            self.command_mode, self.unit, topic, self.step, self.limit,
        )

    def clamp(self, value):
        return max(-self.limit, min(self.limit, value))

    def publish(self):
        msg = Float32MultiArray(data=self.values)
        self.pub.publish(msg)

    def status_line(self):
        cells = []
        for i, v in enumerate(self.values):
            marker = ">" if i == self.selected else " "
            cells.append("{}m{}:{:+.3f}".format(marker, i + 1, v))
        return "[" + self.unit + "] " + "  ".join(cells)

    def print_help(self):
        print(HELP.format(sel=self.selected + 1))

    def handle_key(self, key):
        """Returns False to request shutdown."""
        if key in "1234567":
            self.selected = int(key) - 1
        elif key in ("w", "+"):
            self.values[self.selected] = self.clamp(self.values[self.selected] + self.step)
            self.publish()
        elif key in ("s", "-"):
            self.values[self.selected] = self.clamp(self.values[self.selected] - self.step)
            self.publish()
        elif key == "r":
            self.values[self.selected] = 0.0
            self.publish()
        elif key == " ":
            self.values = [0.0] * N_MOTORS
            self.publish()
        elif key == "h":
            self.print_help()
        elif key == "q":
            return False
        return True

    def zero_and_publish(self):
        self.values = [0.0] * N_MOTORS
        self.publish()


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rospy.init_node("keyboard_teleop")

    if not sys.stdin.isatty():
        rospy.logerr("keyboard_teleop requires an interactive terminal (stdin is not a tty)")
        return

    teleop = KeyboardTeleop()
    teleop.print_help()

    settings = termios.tcgetattr(sys.stdin)
    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key == "\x03":  # Ctrl-C
                break
            if key:
                if not teleop.handle_key(key):
                    break
                print(teleop.status_line() + "   ", end="\r")
                sys.stdout.flush()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.zero_and_publish()
        print("\nkeyboard_teleop: zeroed all motors, exiting.")


if __name__ == "__main__":
    main()
