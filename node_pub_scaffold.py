#!/usr/bin/env python
"""
    Author Harry Terkanian

    January 12, 2020; revised January 23,2020

    Scaffold for node to publish topic /student/<name>
"""


import rospy
from std_msgs.msg import String


class MyNode:


    def __init__(self, first_name):
        #=====initialization============================================

        #=====instance variables========================================
        self.name = first_name

        #=====ROS topic subscriptions and publications==================
        self.pub_name =rospy.Publisher(
                "/student/" + self.name,
                String,
                queue_size = 5
                ) 


#=====Execution begins here=============================================
if __name__ == "__main__":
    node = MyNode("Dave")
    rospy.init_node(node.name)
    rospy.spin()
