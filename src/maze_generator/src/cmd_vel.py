#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import numpy as np

class Keyboard(object):
    def __init__(self):
        topic = '/keys'
        self.subs = rospy.Subscriber(topic, String, self.callback)
    
    def callback(self, msg):
        global key
        key = msg.data

def updateMessage(msg, linear_x, linear_y, angular_z):
    # Asignar los valores al mensaje
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.linear.z = 0.0

    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angular_z

    return msg

if __name__ == '__main__':
    
    # Start node
    rospy.init_node("cmd_vel")

    # Define Publisher
    topic = '/cmd_vel'
    pub = rospy.Publisher(topic, Twist, queue_size=10)

    # Keyboard
    keyboard = Keyboard()
    # Create empty message instance
    twist_msg = Twist()

    # Rate
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # Update speed values
        if key == "w":
            twist_msg = updateMessage(twist_msg, 0.26, 0.0, 0.0)
        elif key == "a":
            twist_msg = updateMessage(twist_msg, 0.26, 0.26, 0.0)
        elif key == "s":
            twist_msg = updateMessage(twist_msg, 0.0, 0.0, 0.2)
        elif key == "d":
            twist_msg = updateMessage(twist_msg, 0.0, 0.0, -0.2)
        elif key == " ":
            twist_msg = updateMessage(twist_msg, 0.0, 0.0, 0.0)
        
        # Publish updated message
        pub.publish(twist_msg)

        # Wait
        rate.sleep()
