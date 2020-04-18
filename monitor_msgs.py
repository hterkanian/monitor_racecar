#!/usr/bin/env python

"""
    Author Harry Terkanian

    January 12, 2020; revised January 23, 2020, January 28, 2020,
    January 29, 2020, January 30, 2020, February 2, 2020, February 5, 2020,
    April 17, 2020

    Unpickle a list of students (a set object) who have previously 
    completed the assignment from file in current directory named in 
    self.student_file.

    Obtain a list of topics currently published in namespace /student. 

    NOTE: Currently processes only String and Float32 messages.
        See: self.student_callback()

    Add any new student(s) who are publishing to the set students who 
    have completed the assignment.
  
    On termination print updated list of students and serialize the 
    set to self.student_file.

    Output (on termination):
        1.  serialized list of students completing assignment in 
            self.student_file; and
        2.  student list to stdout.

    Default namespace is '/student'
    Default self.student_file is  assignment_3.pkl

    Terminate script by publishing any string to /hst/control topic.
"""


import os.path
import pickle
import sys
import rospy
from std_msgs.msg import String, Float32

 
class InventoryTopics(object):
    """Inventory student published topics."""

    def __init__(
            self, 
            default_file='assignment_3.pkl', 
            default_n_space='/student'
    ):
        # =====state class attributes =================================
        self.student_file = default_file
        self.default_namespace = default_n_space
        self.students = self.get_existing_students(self.student_file)
        self.topic_list = self.get_topics(self.default_namespace)
        self.string_msg = String
        self.float_msg = Float32
        self.control_topic = '/hst/control'

        # =====ROS subscribers=========================================
        self.gen_subscribers(self.topic_list)

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
            with open(t_file, 'rb') as f_in:
                students = pickle.load(f_in)
        else:
            students = set()
        print('\n\nStudents who have previously completed this assignment:')
        for student in students:
            print(student)
        return students


    def process_new_topics(self, namespace):
        """Update topic list."""
        current_topics = self.get_topics(namespace)
        new_topics = []
        for topic in current_topics:
            if topic not in self.topic_list:
                new_topics.append(topic)
                self.topic_list.append(topic)
        if len(new_topics) > 0:
            self.gen_subscribers(new_topics)


    def get_topics(self, topic_namespace):
        '''topic_list containing published topics in self.topic_namespace'''
        topic_list = []
        raw_topics = rospy.get_published_topics(namespace=topic_namespace)
        for item in raw_topics:
            msg_type = item[1].split('/')[-1]
            topic_list.append((item[0], msg_type))
        return topic_list


    def gen_subscribers(self, topics):
        '''generate rospy.Subscriber for each topic in topics'''
        for topic in topics:
            if topic[1] == 'String':
                msg_class = self.string_msg
            elif topic[1] == 'Float32':
                msg_class = self.float_msg
            else:
                print('Unknown message class: ', topic[1])
                return
            rospy.Subscriber(
                topic[0], 
                msg_class,  
                callback=self.student_callback,
                callback_args=topic, 
                queue_size=1
            )


    def extract_students_from_topic(self, topic):
        '''extract list of student name(s) from a topic.'''
        raw_students = topic[0].split('/')[2]
        students_in_topic = raw_students.split('_')
        result = []
        for item in students_in_topic:
            if item != 'and':
                result.append(item)
        return result


    def new_student_check(self, students_in_topic):
        '''True if a new student is named in the topic.'''
        result = False
        for student in students_in_topic:
            if student not in self.students:
                result = True
        return result


    def student_callback(self, msg, topic):
        '''process each student message'''
        students = self.extract_students_from_topic(topic)
        new_student = self.new_student_check(students)
        if new_student:    # new_student == True if a new student in topic
            if topic[1] == 'String' or topic[1] == 'Float32':
                student_list = ''
                for item in students:
                    new_item = ' ' + item
                    student_list += new_item
                print('Message from: ' + student_list)
                if topic[1] == 'String':
                    msg_text = msg.data
                else:
                    msg_text = str(msg.data)
                print(msg_text)
                ans = raw_input("[y]es if OK, otherwise [n]o: ")
                if 'y' in ans.lower():
                    # add students publishing this message to self.students
                    for student in students:
                        if student != 'and':
                            self.students.add(student)
            else:
                print('Unexpected message type: ' + 
                      topic[1] + 
                      ' received on topic ',
                      topic[0]
                     )
        return


    def control_callback(self, msg):
        '''shutdown node on receipt of any message on /hst/control topic'''
        self.report_and_save(self.students, self.student_file)
        rospy.signal_shutdown('Normal termination')
        sys.exit(0)


    def report_and_save(self, student_list, out_file):
        '''print student list and serialize to self.student_file'''
        print('\n\nCurrent list of students who have completed ' +
              'the assignment as saved in file: ' +
              self.student_file
             )
        for student in student_list:
            print(student)
        with open(out_file, 'wb') as f_out:
            pickle.dump(student_list, f_out)


#=====Execution entry point============================================
if __name__ == "__main__":
    rospy.init_node('student_monitor')
    node = InventoryTopics()
    while not rospy.is_shutdown():
        node.process_new_topics(node.default_namespace)
        rospy.sleep(10)
