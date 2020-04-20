### monitor_racecar
Beta versions of Python scripts for monitoring student ROS code communications with master.  Scripts to monitor node creaton, topics
published to, and messages published to topics.  Histories are maintained as pickled Python set objects.

#### Environment and Limitations
Since roscore likely will be run on a robot or a computer different from the student's computer, environment variables identifying the location of roscore and the IP address of the student machine must be set:

      1.  export ROS_MASTER_URI=http://<roscore IP address>:11311
      2.  export ROS_IP=<localhost IP address>
      
  NOTE:  Issues remain when roscore and the student computers are on different networks, for example wher the student code is being run on a virtual machine.

#### monitor_nodes.py
Monitors the creation of nodes in a specific namespace e. g. beginning with 'student' and containing the names of one or more students. Valid node names include 'student_Bob_and_Mary' or 'student_Bob_Mary'   Node names are split on the underscore and the token 'student' is discarded.  student names should contain no whitespace or underscores.  Use first, last or nicknames.  Sutdent names are extracted from the node name and added to an existing Python set object of students in order to maintain a runnig total of all students who have successfully created a ROS node.  If there is an existing serialized set object the names are added; if not, a new set object is created.  On receipt of a termination command (any string message published to the control topic) the set object is serialized using pickle.

#### monitor_pub.py
Monitors the creation of a rospy publisher object publishing to a topic in namespace '/student'  Topic names should be of the form '/student/<student name(s)/<topic_name>  Student names are in the same format as for monitor_nodes.py above.  The '/student' and '/<topic_name>' portions of the mesage topic are discarded.  Student names are maintained in a serialized (using pickle) Python set object.  If and existing set exists, it is unpickled, if not, a new set object is created.  New student names are added to the set object.  On receipt of a termination command (as above) the set is serialized and the script terminates.

#### monitor_msgs.py
Monitors the successful publication of String and Float32 message types by students.  Subscribes to published topics in namespace '/student.'  Topics are as above: '/student/<student name(s)>/<topic_name>'  Currently only String and Float32 message types are supported. Extracts and maintians a list studens successfully publishing to topics satisfying these constraints in the same manner as monitor_pub.py.  Termination is as in monitor_pub.py.

##### other files
Other files in this repository are test, demonstration, or studnet framework files.
