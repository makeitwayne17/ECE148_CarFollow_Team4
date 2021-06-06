#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CameraInfo

rospy.init_node('camera_info', anonymous=True)

pub = rospy.Publisher('/camera_rect/camera_info', CameraInfo, queue_size=10)
rate = rospy.Rate(60)
while not rospy.is_shutdown():
    q=CameraInfo()

    q.header.frame_id='usb_cam'
    q.height=480
    q.width=640
    q.D=[-0.305375755798262, 0.08733107743324078, 0.0008733245806369092, -0.0019880226564317873, 0.0]
    q.K=[378.7228989308103, 0.0, 313.0386974744686, 0.0, 380.9286533645945, 233.16865485340253, 0.0, 0.0, 1.0]
    q.R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    q.P=[285.3359680175781, 0.0, 305.8216940682214, 0.0, 0.0, 327.4324035644531, 230.98217282522455, 0.0, 0.0, 0.0, 1.0, 0.0]
    q.binning_x=0
    q.binning_y=0
    q.roi.x_offset=0
    q.roi.y_offset=0
    q.roi.height=0
    q.roi.width=0
    q.roi.do_rectify=False
    pub.publish(q)
    rate.sleep()
