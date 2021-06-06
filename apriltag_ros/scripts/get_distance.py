#! /usr/bin/env python
 
import rospy # Import the Python library for ROS
from std_msgs.msg import Int32, Float32 # Import the Int32 message from the std_msgs package
import geometry_msgs.msg
#import tf2_msgs
from tf2_msgs.msg import TFMessage

#import tf2

global steering_float, throttle_float
steering_float = Float32()
STEERING_TOPIC_NAME = '/steering'

def callback(msg): # Define a function called 'callback' that receives a parameter named 'msg'
  z_val = msg.transforms[-1].transform.translation.z
  x_val = msg.transforms[-1].transform.translation.x
  steering_float = x_val
  steering_pub.publish(steering_float)
  print("side value:",x_val,"            front value:",z_val)
  #print(msg.translation.x) # Print the value 'data' inside the 'msg' parameter
 
rospy.init_node('topic_subscriber') # Initiate a Node called 'topic_subscriber'

steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1) 
sub = rospy.Subscriber('tf', TFMessage, callback) # Create a Subscriber object that will listen to the /counter topic and will call the 'callback' function each time it reads something from the topic
#apriltag_ros/AprilTagDetectionArray tag_detections
 
rospy.spin() # Create a loop that will keep the program in execution
