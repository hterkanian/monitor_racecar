#!/usr/bin/python
"""
Author Harry Terkanian

January 12, 2020
Most recent revision: April 17, 2020

Publishes a float.
"""


import rospy
from std_msgs.msg import Float32

class PubFloat(object):
    """Sample publisher to publish float to custom topic."""


    def __init__(self):

        # =====state class attributes =================================
        self.pub = rospy.Publisher(
            '/student/harry_and_bill/float', 
            Float32, 
            queue_size=1
        )

    def process(self):
        """main loop."""
        self.pub.publish(32.0)


#=====Execution begins here============================================
if __name__ == "__main__":
    node = PubFloat()
    rospy.init_node('doc_publisher')
    while True:
        node.process()
        rospy.sleep(10)
