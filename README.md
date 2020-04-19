### monitor_racecar
Beta versions of Python scripts for monitoring student ROS code communications with master.  Scripts to monitor node creaton, topics
published to, and messages published to topics.  Histories are maintained as pickled Python set objects.

#### monitor_nodes.py
Monitors the creation of nodes in a specific namespace e. g. beginning with '/student' and containing the names of one or more students e. g. '''rospy.init_node('/student_Bob_and_Mary')'''  Extracts sutdent names and adds them to an existing set of students maintained in a pickled Python set object.

#### monitor_pub.py
Monitors the creation of a rospy publisher object publishing to topics in namespace '/student'  Extracts and lists student names in the same manner as '''monitor_nodes.py'''

#### monitor_msgs.py
Monitors messages published to topics in namespace '/student'  Extracts and lists student names in the same manner as '''monitor_nodes.py'''

##### other files
Other files in this repository are test, demonstration, or studnet framework files.
