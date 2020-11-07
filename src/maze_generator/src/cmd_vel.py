#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, os, tty, termios

msg = """
Control Your Micromouse!
---------------------------
Moving around:
        w
   a    s    d

space key : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def updateMessage(msg, linear_x, angular_z):
    # Asignar los valores al mensaje
    msg.linear.x = linear_x
    msg.linear.y = 0.0
    msg.linear.z = 0.0

    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angular_z

    return msg

if __name__ == '__main__':
    
    # Start node
    rospy.init_node("cmd_vel")
    settings = termios.tcgetattr(sys.stdin)
    # Define Publisher
    topic = '/cmd_vel'
    pub = rospy.Publisher(topic, Twist, queue_size=10)

    # Create empty message instance
    twist_msg = Twist()

    # Rate
    rate = rospy.Rate(100)

    print(msg)
    while not rospy.is_shutdown():
        # Update speed values
        key = getKey()
        if key == "w":
            twist_msg = updateMessage(twist_msg, 0.26, 0.0)
        elif key == "a":
            twist_msg = updateMessage(twist_msg, 0.0, 0.2)
        elif key == "s":
            twist_msg = updateMessage(twist_msg, -0.26, 0.0)
        elif key == "d":
            twist_msg = updateMessage(twist_msg, 0.0, -0.2)
        elif key == " ":
            twist_msg = updateMessage(twist_msg, 0.0, 0.0)
        elif (key == '\x03'): # Stop with ctrl+c
            break
        # Publish updated message
        pub.publish(twist_msg)
        # Wait
        rate.sleep()
