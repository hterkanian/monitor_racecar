#!/usr/bin/python
"""
    Author Harry Terkanian

    January 12, 2020
    Most recent revision: none

    Publishes a doc string.
"""


import rospy
from std_msgs.msg import Float32

class PubDocstring:


    def __init__(self):

        # =====state class attributes =================================
        self.pub = rospy.Publisher('/student/harry_and_bill/doc', Float32, queue_size = 1)

    def process(self):
        self.pub.publish(32.0)


#=====Execution begins here============================================
if __name__ == "__main__":
    node = PubDocstring()
    rospy.init_node('doc_publisher')
    while True:
        node.process()
        rospy.sleep(10)
