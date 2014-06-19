#!/usr/bin/env python
# Written By: Tigist

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
  pub_topic = rospy.Publisher('webcam_topic',Image)
  rospy.init_node('webcam_node')
  
  im_bridge = CvBridge()
  #Get the images from the webcam
  webcam = cv2.VideoCapture(0)
  
  pub_rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    success, im = webcam.read()
    bitmap = cv2.cv.CreateImageHeader((im.shape[1], im.shape[0]), cv2.cv.IPL_DEPTH_8U, 3)
    cv2.cv.SetData(bitmap, im.tostring(), im.dtype.itemsize * 3 * im.shape[1])
    #Convert and publish
    pub_topic.publish(im_bridge.cv_to_imgmsg(bitmap, 'bgr8'))
    pub_rate.sleep()
  
  webcam.release()  


