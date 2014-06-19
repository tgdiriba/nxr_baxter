#!/usr/bin/env python
# Written By: Tigist

import rospy
from ar_track_alvar.msg import CartesianCoordinate

if __name__ == '__main__':
  rospy.init()
  
  cartesian_topic = Publisher('cartesian_topic', CartesianCoordinate)
  
  cartesian_rate = Rate(10)
  
  while not rospy.is_shutdown():
    cartesian_coordinate = CartesianCoordinate(4,5,6)
    cartesian_topic.publish(cartesian_coordinate))
    cartesian_rate.sleep()

