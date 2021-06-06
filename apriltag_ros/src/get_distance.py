#! /usr/bin/env python
 
import rospy # Import the Python library for ROS
from std_msgs.msg import Int32 # Import the Int32 message from the std_msgs package
 
def callback(msg): # Define a function called 'callback' that receives a parameter named 'msg'
  print msg.data # Print the value 'data' inside the 'msg' parameter
 
rospy.init_node('topic_subscriber') # Initiate a Node called 'topic_subscriber'
 
sub = rospy.Subscriber('tf', AprilTagDetectionArray, callback) # Create a Subscriber object that will listen to the /counter topic and will call the 'callback' function each time it reads something from the topic
#apriltag_ros/AprilTagDetectionArray tag_detections
 
rospy.spin() # Create a loop that will keep the program in execution
