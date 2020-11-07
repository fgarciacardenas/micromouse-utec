#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, os, tty, termios

infoMsg = """
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

# Python input from terminal
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def updateMessage(msg, linear_x, angular_z):
    # Assign values
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

    # Information of how to control the robot
    print(infoMsg)

    # Increase and decrease speed value
    step_linear = 0.02
    step_angular = 0.2
    # Linear and angular velocities
    linear = 0.0
    angular = 0.0
    # Top velocities
    top_linear = 0.26
    top_angular = 5
    
    while not rospy.is_shutdown():
        # Update speed values
        key = getKey()
        if key == "w":
            linear = linear + step_linear
        elif key == "a":
            angular = angular - step_angular
        elif key == "s":
            linear = linear - step_linear
        elif key == "d":
            angular = angular + step_angular
        elif key == " ":
            linear = 0.0
            angular = 0.0
        elif (key == '\x03'): # Stop with ctrl+c
            break

        # Limit velocity values
        if abs(linear) > top_linear:
            linear = top_linear
        
        if abs(angular) > top_angular:
            angular = top_angular

        twist_msg = updateMessage(twist_msg, linear, angular)
        
        # Publish updated message
        pub.publish(twist_msg)
        
        # Wait
        rate.sleep()
