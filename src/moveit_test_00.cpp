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

	move_group_interface::MoveGroup left_group("left_arm");
	move_group_interface::MoveGroup right_group("right_arm");
	left_group.setRandomTarget();
	right_group.setRandomTarget();
	
	left_group.move();
	right_group.move();
	
	ros::waitForShutdown();
}
