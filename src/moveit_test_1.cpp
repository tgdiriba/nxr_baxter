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

	map<string, double> left_arm_joints;
	left_arm_joints["left_e0"] = 0.5;
	left_arm_joints["left_e1"] = 0.5;
	left_arm_joints["left_s0"] = 0.5;
	left_arm_joints["left_s1"] = 0.5;
	left_arm_joints["left_w0"] = 0.5;
	left_arm_joints["left_w1"] = 0.5;
	left_arm_joints["left_w2"] = 0.5;
	left_arm_joints["left_hand"] = 0.5;
	left_arm_joints["left_endpoint"] = 0.5;

	move_group_interface::MoveGroup left_group("left_arm");
	// left_group.setRandomTarget();
	
	left_group.setJointValueTarget(left_arm_joints);
	left_group.move();
	
	ros::waitForShutdown();
}
