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

throttle_dic = {'previous_throttle': 0.0}

# dictionaries for time steps and error
error_dic = {'previous_error': 0.0, 'normalized_error': 0.0}
time_dic = {'previous_time': 0.0, 'current_time': 0.0}

STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'

howFarAwayWeWant = 0.3


def line_follower(data):
  normalized_error = None
  distance = data - howFarAwayWeWant
  if distance < -.2: # and distance < 0   #E brake logic
    # throttle_float = throttle_dic.get('previous_throttle')*
    throttle_pub.publish(-0.99)
    #rospy.sleep(2)
    throttle_pub.publish(0.0)
    throttle_pub.publish(0.0)
    print("Too close ", distance)

  # elif distance == -1:
  #     normalized_error = None
  else:
      print("We at: ",distance)
      error_z = float(distance)     #distance already modified, idealy at zero already
      normalized_error = float(error_z / 3)   # distance of 5 is our max distance out
      if normalized_error > 1:
        normalized_error = 1  # distance of 5 is our max distance out

      t = rospy.Time.from_sec(time.time())
      current_time = t.to_sec()  # floating point
      time_dic['current_time'] = current_time
      previous_time = time_dic.get('previous_time')
      delta_time = current_time - previous_time
      previous_error = error_dic.get('previous_error')
      de_dt = (normalized_error - previous_error) / delta_time
      integral_error = 0
      integral_error = integral_error + normalized_error * delta_time

      kp = 3.0
      kd = 0.000001
      ki = 0.000000

      print("previous error: ", previous_error)
      print("previous error: ", error_z)
      print("previous time: ", time_dic['previous_time'])
      print("current time: ", time_dic['current_time'])
      time_dic['previous_time'] = current_time
      error_dic['previous_error'] = error_z

  # rpm_int = 1500
  if normalized_error is None:
      pass
  else:
      control_signal = kp * normalized_error + kd * de_dt + ki * integral_error
      print("control_signal: ", control_signal)
      throttle_float = float(control_signal)

      # if steering_float < -1:
      #     steering_float = -1
      # elif steering_float > 1:
      #     steering_float = 1
      # elif steering_float < 0:
      #     steering_float = steering_float*1.4
      # elif steering_float > 0:
      #     steering_float = steering_float*1.4
      # else:
      #     pass

      throttle_pub.publish(throttle_float)
      throttle_dic['previous_throttle'] = throttle_float
  # throttle_pub.publish(rpm_int)


def callback(msg): # Define a function called 'callback' that receives a parameter named 'msg'
  z_val = msg.transforms[-1].transform.translation.z
  x_val = msg.transforms[-1].transform.translation.x
  steering_float = x_val*0.85
  
  if(steering_float >= 1.0):
    steering_float = 0.99
  elif(steering_float <= -1.0):
    steering_float = -0.99
  line_follower(z_val)

  
  steering_pub.publish(steering_float)
  # throttle_pub.publish(throttle_float)
  print("side value:",x_val,"            front value:",z_val)
  #print(msg.translation.x) # Print the value 'data' inside the 'msg' parameter
 
rospy.init_node('topic_subscriber') # Initiate a Node called 'topic_subscriber'

throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1) 

sub = rospy.Subscriber('tf', TFMessage, callback) # Create a Subscriber object that will listen to the /counter topic and will call the 'callback' function each time it reads something from the topic
#apriltag_ros/AprilTagDetectionArray tag_detections
 
rospy.spin() # Create a loop that will keep the program in execution

