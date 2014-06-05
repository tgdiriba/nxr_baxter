#!/usr/bin/env python

import rospy
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from geometry_msgs import Pose

class AlvarTracker():

	def __init__(self):
		rospy.init_node("nxr_ar_tracker")
		rospy.Subscriber("ar_pose_marker",AlvarMarkers,self.ar_tracker_sub)
		self.pub = rospy.Publisher("nxr_ar_tracked",Pose)
		
		self.largest_marker = AlvarMarker()
		self.tracked = Pose()
		
		self.ar_tracker_pub()
	
	def ar_tracker_sub(self,marker_set):
		self.tracked = max(marker_set,key=lambda marker: marker.confidence).pose.pose
	
	def ar_tracker_pub(self):
		# Track the object with the highest data confidence
		self.ar_rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.pub.publish(self.tracked)
			rospy.spin()
	
if __name__ == '__main__':
	try:
		tracker = AlvarTracker()
	except Exception, e:
		print e