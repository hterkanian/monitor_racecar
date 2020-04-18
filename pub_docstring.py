#!/usr/bin/python
"""
Author Harry Terkanian

January 12, 2020
Most recent revision: April 17, 2020

Publishes a doc string.
"""


import rospy
from std_msgs.msg import String

class PubDocstring(object):
    """Publishes __doc__"""

    def __init__(self):

        # =====state class attributes =================================
        self.pub = rospy.Publisher(
            '/student/harry_and_bill/doc', 
            String, 
            queue_size=1
        )

    def process(self):
        """Publish module __doc__"""
        self.pub.publish(__doc__)


#=====Execution entry point============================================
if __name__ == "__main__":
    node = PubDocstring()
    rospy.init_node('doc_publisher')
    while True:
        node.process()
        rospy.sleep(10)
