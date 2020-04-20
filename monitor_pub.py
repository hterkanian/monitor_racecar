#!/usr/bin/env python

"""
    Author Harry Terkanian

    January 12, 2020; revised January 23, 2020, January 28, 2020,
    January 29, 2020, January 30, 2020, February 2, 2020, February 5, 2020,
    April 17-20, 2020

    Unpickle a list of students (a Python set object) who have previously 
    completed the assignment from file in current directory named in 
    self.student_file.

    Obtain a list of topics currently published in namespace /student. 

    Add any new student(s) who are publishing to the set students who 
    have previously published.
  
    On termination print updated list of students and serialize the 
    set to self.student_file.

    Environment: Environment variables identifying the location of roscore
    and the IP address of this machine must be set:
    1.  export ROS_MASTER_URI=http://<roscore IP address>:11311
    2.  export ROS_IP=<localhost IP address>

    Input:  none

    Output (on termination):
        1.  serialized list of students completing assignment in 
            self.student_file; and
        2.  student list to stdout.

    Defaults: (set when node is initialized)
    1.  Default namespace: default_n_space = '/student'
    2.  Default student file: default_file = assignment_2.pkl
    3.  Default control topic: self.control_topic = '/hst/control'

    Reset: 
    Clear the list of students delete the file named in default_file

    Termination:
    Ppublish any string message to /hst/control topic.
"""


import os.path
import pickle
import sys
import rospy
from std_msgs.msg import String


class InventoryTopics(object):
    """Maintain an inventory of student published topics."""

    def __init__(
            self, 
            default_file='assignment_2.pkl', 
            default_n_space='/student'
    ):
        # =====state class attributes =================================
        self.student_file = default_file
        self.default_namespace = default_n_space
        self.students = self.get_existing_students(self.student_file)
        self.string_msg = String
        self.control_topic = '/hst/control'

        # =====ROS subscribers=========================================
        # control chanel subscriber
        rospy.Subscriber(
            self.control_topic, 
            String, 
            self.control_callback, 
            queue_size=1
        )


    def get_existing_students(self, t_file):
        '''open self.student_file if present, if not, return empty set'''
        if os.path.exists(self.student_file):
            with open(t_file, 'rb') as topic_f:
                students = pickle.load(topic_f)
        else:
            students = set()
        print('\n\nStudents who have previously completed this assignment are:')
        for student in students:
            print(student)
        return students


    def get_topics(self, topic_namespace):
        '''topic_list containing published topics in self.topic_namespace'''
        new_topic_list = []
        raw_topics = rospy.get_published_topics(namespace=topic_namespace)
        for item in raw_topics:
            msg_type = item[1].split('/')[-1]
            new_topic_list.append((item[0], msg_type))
        return new_topic_list


    def extract_students_from_topic(self, topic_in):
        '''extract list of student name(s) from a topic.'''
        raw_students = topic_in[0].split('/')[2]
        students_publishing = raw_students.split('_')
        result = []
        for item in students_publishing:
            if item != 'and':
                result.append(item)
        return result


    def new_student_check(self, students_pub_topic):
        '''True if a new student is named in the topic.'''
        result = []
        for student in students_pub_topic:
            if student not in self.students:
                result.append(student)
                print('New student name in topic: ' + student)
        return result


    def control_callback(self, msg):
        '''shutdown node on receipt of any message on /hst/control topic'''
        self.report_and_save(self.students, self.student_file)
        rospy.signal_shutdown('Normal termination')
        sys.exit(0)


    def report_and_save(self, student_list, out_file):
        '''print student list and serialize to self.student_file'''
        print('\n\nCurrent list of students who have completed ' +
              self.student_file)
        for student in student_list:
            print(student)
        with open(out_file, 'wb') as out_f:
            pickle.dump(student_list, out_f)


#=====Execution entry point============================================
if __name__ == "__main__":
    rospy.init_node('student_pub')
    inventory_node = InventoryTopics()
    while not rospy.is_shutdown():
        topic_list = inventory_node.get_topics(inventory_node.default_namespace)
        for topic in topic_list:
            students_in_topic = inventory_node.extract_students_from_topic(topic)
            new_students = inventory_node.new_student_check(students_in_topic)
            if len(new_students) > 0:
                for new_student in new_students:
                    inventory_node.students.add(new_student)
        rospy.sleep(10)
