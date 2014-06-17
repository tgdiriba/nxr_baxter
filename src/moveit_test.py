#!/usr/bin/env/ python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':
	print '=== Starting Tutorial Setup ==='
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
	
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander('left_arm')
	
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
	
	print '=== Waiting for RVIZ ==='
	rospy.sleep(10)
	print '=== Starting Tutorial ==='
	
	print '=== Reference Frame: %s ===' % group.get_planning_frame()i
	
	print '=== Robot Groups:'
	print robot.get_group_names()
	
	print '=== Robot State ==='
	print robot.get_current_state()
	
	# Planning a Pose goal
	print '=== Generating plan 1'
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1.0
	pose_target.position.x = 0.7
	pose_target.position.y = -0.05
	pose_target.position.z = 1.1
	group.set_pose_target(pose_target)
