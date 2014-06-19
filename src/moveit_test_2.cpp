#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <map>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_test_0", ros::init_options::AnonymousName);
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup left_hand("left_hand");
	move_group_interface::MoveGroup right_hand("right_hand");

	geometry_msgs::Pose l_pose, r_pose;

	l_pose.position.x = -0.2;
	l_pose.position.y = 0.2;
	l_pose.position.z = 0.2;
	l_pose.orientation.x = 0.0;
	l_pose.orientation.y = 0.0;
	l_pose.orientation.z = 0.0;
	l_pose.orientation.w = 0.0;

	r_pose.position.x = 0.2;
	r_pose.position.y = 0.2;
	r_pose.position.z = 0.2;
	r_pose.orientation.x = 0.0;
	r_pose.orientation.y = 0.0;
	r_pose.orientation.z = 0.0;
	r_pose.orientation.w = 0.0;
	
	left_hand.setPoseTarget(l_pose, "left_hand_eef");
	right_hand.setPoseTarget(r_pose, "right_hand_eef");

	ros::waitForShutdown();
}
