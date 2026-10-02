#!/usr/bin/env python3
"""
legacy_task_space_teleop.py
----------------------------
Early ROS 1 prototype for task-space keyboard teleoperation of the 6-DOF arm
(x, y, z, phi, theta, psi). It was used during the stepper/serial-control phase
of the project, before the switch to the ROS 2 + micro-ROS architecture
(see ../keyboard_teleop.py for the current, joint-space teleop node).

Keyboard layout and the PublishThread structure are adapted from the
BSD-3-Clause licensed `teleop_twist_keyboard` package
(https://github.com/ros-teleop/teleop_twist_keyboard,
Copyright (c) 2015, PAL Robotics, S.L.). The task-space (x, y, z, phi, theta,
psi) mapping and the integration with this robot's serial/ROS bridge were
written by Luca Obwegs.

Controls
--------
    u i o / j k l / m , .   -> linear x/y/z direction keys (see `msg` below)
    U I O / J K L / M < >   -> holonomic strafing variants
    t / b                   -> up / down (+z / -z)
    q                        -> increase speed scaling by 10%
    CTRL-C                   -> quit

Usage
-----
    rosrun <your_package> legacy_task_space_teleop.py
    (requires roscore and the rosserial bridge to the Arduino Mega running,
    see ros_instructions.txt)

Author: Luca Obwegs
License: GPL-3.0-only for the project-specific parts (see ../LICENSE);
         BSD-3-Clause for the parts adapted from teleop_twist_keyboard.
"""

from __future__ import print_function

import select
import sys
import termios
import threading
import tty

import roslib
roslib.load_manifest("teleop_twist_keyboard")
import rospy
from geometry_msgs.msg import Twist

msg = """
Reading from the keyboard and publishing task-space jog commands!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q : increase speed scaling by 10%

CTRL-C to quit
"""

moveBindings = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

speedBindings = {
    "q": (1.1, 1.1),
}


class PublishThread(threading.Thread):
    """Publishes the current task-space jog command at a fixed rate."""

    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher("/arm_task_space_cmd", Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        self.timeout = 1.0 / rate if rate != 0.0 else None
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i = (i + 1) % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, phi, theta, psi):
        self.condition.acquire()
        self.x, self.y, self.z = x, y, z
        self.phi, self.theta, self.psi = phi, theta, psi
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            twist.linear.x = self.x * self.theta
            twist.linear.y = self.y * self.theta
            twist.linear.z = self.z * self.theta
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.phi * self.psi

            self.condition.release()
            self.publisher.publish(twist)


def get_key(settings, key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("legacy_task_space_teleop")

    theta = rospy.get_param("~theta", 0.5)
    psi = rospy.get_param("~psi", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = y = z = phi = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, phi, theta, psi)

        print(msg)
        while True:
            key = get_key(settings, key_timeout)
            if key in moveBindings:
                x, y, z, phi = moveBindings[key]
            elif key in speedBindings:
                theta *= speedBindings[key][0]
                psi *= speedBindings[key][1]
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                if key == "" and x == 0 and y == 0 and z == 0 and phi == 0:
                    continue
                x = y = z = phi = 0
                if key == "\x03":  # Ctrl+C
                    break
            pub_thread.update(x, y, z, phi, theta, psi)

    except Exception as error:
        print(error)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
