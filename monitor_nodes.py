#!/usr/bin/env python
"""
Harry Sarkis Terkanian

January 20, 2020; revised January 22, 2020, January 30, 2020, 
February 1, 2020, February 2, 2020, February 4, 2020, February 5, 2020,
April 17-20, 2020

A script to check for and log student created nodes with node 
names in the form 'student_<name(s)>' and maintain and print a 
cumulataive list. 

Examples of valid node names: 'student_JacK_Jill' or 'student_Jack_and_Jill'

Acquire a list of currently running nodes.  Student nodes should be
named as above so this script can identify a node with the students 
running it.
    
Existing list of student nodes, if any, assumed in a pickled set 
    object in file named by default_file variable located in 
    the current working directory.

Environment: Environment variables identifying the location of roscore 
and the IP address of this machine must be set:
    1.  export ROS_MASTER_URI=http://<roscore IP address>:11311
    2.  export ROS_IP=<localhost IP address>

Input:  none

Output:
    1.  any new nodes are added to student set and pickled in a 
        file in the current directory nameed in 
        the default_file variable and located in current directory.
    2.  print a list of students to stdout.


Termination:
    Publish any string message to topic set by self.control_topic variable.

Defaults: (Set when node object created)
    1.  File containing names of students: default_file = 'assignment_1.pkl'
    2.  Default node namespace begins with value in variable 
        default_n_space = 'student'
    3.  Default control topic: self.control_topic = '/hst/control'

Reset:
    To clear the list delete file named in default_file
"""

import os.path
import pickle
import sys
import rospy
import rosnode
from std_msgs.msg import String


class InventoryNodes(object):
    """Log student created nodes."""

    def __init__(
            self,
            default_file='assignment_1.pkl',
            default_n_space='student'
    ):
        # =====class state attributes======================================
        self.student_file = default_file
        self.default_namespace = default_n_space
        self.control_topic = '/hst/control'
        self.students = self.get_existing_students(self.student_file)

        # =====ROS subscribers=============================================
        # =====control channel subscriber
        rospy.Subscriber(
            self.control_topic,
            String,
            self.control_callback,
            queue_size=1
        )


    def get_existing_students(self, t_file):
        '''open self.student_file if present, if not, return empty set'''
        if os.path.exists(self.student_file):
            with open(t_file, 'rb') as f_in:
                students = pickle.load(f_in)
        else:
            students = set()
        print('\n\nStudents who have previously completed this assignment:')
        for student in students:
            print(student)
        return students


    def control_callback(self, msg):
        '''shutdown node on receipt of any message on /hst/control topic'''
        self.report_and_save(self.students, self.student_file)
        rospy.signal_shutdown('Normal termination')
        sys.exit(0)


    def report_and_save(self, student_list, out_file):
        '''print student list and serialize to self.student_file'''
        print('\n\nCurrent list of students who have completed ' +
              'the assignement and saved in file: ' +
              self.student_file
             )
        for student in student_list:
            print(student)
        with open(out_file, 'wb') as f_out:
            pickle.dump(student_list, f_out)


    def get_nodes(self):
        '''get the list of current nodes, check for new student.'''
        node_list = rosnode.get_node_names()
        for node in node_list:
            tokens = node.split('_')
            if 'student' in tokens[0].lower():
                for token in tokens:
                    if "student" not in token.lower() and \
                            "and" not in token.lower() \
                            and len(token) > 0 \
                            and token not in self.students:
                        print('New student ' + 
                              token + 
                              ' appears in a node name.'
                             )
                        self.students.add(token)


#====Execution entry point=============================================
if __name__ == '__main__':
    rospy.init_node('node_monitor')
    inv_node = InventoryNodes()
    while not rospy.is_shutdown():
        inv_node.get_nodes()
        rospy.sleep(10)
