#!/usr/bin/env python
# Written By: Tigist

import rospy
import cv2
from ar_track_alvar.msg import WebcamInfo

if __name__ == '__main__':
  
  rospy.init_node('info_node')
  info_topic = Publisher('info_topic', WebcamInfo)
  
  info_rate = Rate(10)

  while not rospy.is_shutdown():
    webcam_info = WebcamInfo(1,2,3)
    
    info_topic.publish(webcam_info)
    
    info_rate.sleep()
