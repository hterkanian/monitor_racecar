#!/usr/bin/python
"""
    Author Harry Terkanian

    January 12, 2020
    Most recent revision: none

    Prints a doc string published on topic /student/harry/doc.
"""


import rospy
from std_msgs.msg import String

class PubDocString(object):
    """Subscribes to topic and prints __doc__ string."""

    def __init__(self):
        #=====state class attributes ==================================
        self.topic = '/student/harry/doc'
        self.sub = rospy.Subscriber(self.topic, String, self.process)


    def process(self, msg):
        """Msg callback: print message."""
        print('Publisned on toipc ' + self.topic + ':\n' + msg.data)


#====Execution begins here=============================================
if __name__ == "__main__":
    node = PubDocString()
    rospy.init_node('doc_printer')
    rospy.spin()
