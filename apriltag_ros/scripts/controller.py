#! /usr/bin/env python
 
import rospy # Import the Python library for ROS
from std_msgs.msg import Int32, Float32 # Import the Int32 message from the std_msgs package
import geometry_msgs.msg
#import tf2_msgs
from tf2_msgs.msg import TFMessage
import time

#import tf2

global steering_float, throttle_float
steering_float = Float32()
throttle_float = Float32()

# dictionaries for time steps and error
error_dic = {'previous_error': 0.0, 'normalized_error': 0.0}
time_dic = {'previous_time': 0.0, 'current_time': 0.0}

STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'

def line_follower(data):
  distance = data
  if distance == 0:
          steering_float = 0.0
          steering_pub.publish(steering_float)
      elif centroid == -1:
          normalized_error = None
      else:
          error_x = float(centroid - (width / 2))
          normalized_error = float(error_x / (width / 2))

          t = rospy.Time.from_sec(time.time())
          current_time = t.to_sec()  # floating point
          time_dic['current_time'] = current_time
          previous_time = time_dic.get('previous_time')
          delta_time = current_time - previous_time
          previous_error = error_dic.get('previous_error')
          de_dt = (normalized_error - previous_error) / delta_time
          integral_error = 0
          integral_error = integral_error + normalized_error * delta_time

          kp = 0.4
          kd = 0.000001
          ki = 0.000000

          print("previous error: ", previous_error)
          print("previous error: ", error_x)
          print("previous time: ", time_dic['previous_time'])
          print("current time: ", time_dic['current_time'])
          time_dic['previous_time'] = current_time
          error_dic['previous_error'] = error_x

      rpm_int = 1500
      if normalized_error is None:
          pass
      else:
          control_signal = kp * normalized_error + kd * de_dt + ki * integral_error
          print("control_signal: ", control_signal)
          steering_float = float(control_signal)
          if steering_float < -1:
              steering_float = -1
          elif steering_float > 1:
              steering_float = 1
          elif steering_float < 0:
              steering_float = steering_float*1.4
          elif steering_float > 0:
              steering_float = steering_float*1.4
          else:
              pass
          steering_pub.publish(steering_float)
      throttle_pub.publish(rpm_int)


def callback(msg): # Define a function called 'callback' that receives a parameter named 'msg'
  z_val = msg.transforms[-1].transform.translation.z
  x_val = msg.transforms[-1].transform.translation.x
  steering_float = x_val
  
  if(steering_float >= 1.0):
    steering_float = 0.99
  elif(steering_float <= -1.0):
    steering_float = -0.99
  line_follower(z_val)

  
  steering_pub.publish(steering_float)
  throttle_pub.publish(throttle_float)
  print("side value:",x_val,"            front value:",z_val)
  #print(msg.translation.x) # Print the value 'data' inside the 'msg' parameter
 
rospy.init_node('topic_subscriber') # Initiate a Node called 'topic_subscriber'

throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1) 

sub = rospy.Subscriber('tf', TFMessage, callback) # Create a Subscriber object that will listen to the /counter topic and will call the 'callback' function each time it reads something from the topic
#apriltag_ros/AprilTagDetectionArray tag_detections
 
rospy.spin() # Create a loop that will keep the program in execution
