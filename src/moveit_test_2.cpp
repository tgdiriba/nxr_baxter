#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <map>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>

#define NUM_CHILD_THREADS 1

using namespace cv;
using namespace std;

pthread_t tids[NUM_CHILD_THREADS];

void *image_func(void *thread_data)
{
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	
	Mat image = imread("/home/tgd/ros_ws/src/nxr_baxter/images/lc.jpg", CV_LOAD_IMAGE_UNCHANGED);
	namedWindow("Test OpenCV", CV_WINDOW_AUTOSIZE);

	if(!image.data) {
		ROS_ERROR("Could not find or open the image");
	}
	else {
		imshow("Test OpenCV", image);
		while(ros::ok()) {
			loop_rate.sleep();	
			ros::spinOnce();
		}
	}

	pthread_exit(NULL);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "move_group_interface_test_0", ros::init_options::AnonymousName);
	
	ros::NodeHandle nh;
	ros::Duration(1).sleep();	
	
	ros::AsyncSpinner spinner(1);
	spinner.start();

	pthread_create(&(tids[0]), NULL, image_func, NULL);
	
	move_group_interface::MoveGroup left_hand("left_hand");
	move_group_interface::MoveGroup right_hand("right_hand");

	geometry_msgs::Pose l_pose, r_pose;

	l_pose.position.x = -0.2;
	l_pose.position.y = 0.2;
	l_pose.position.z = 0.2;
	l_pose.orientation.x = 0.577;
	l_pose.orientation.y = 0.577;
	l_pose.orientation.z = 0.577;
	l_pose.orientation.w = 1.0;

	r_pose.position.x = 0.2;
	r_pose.position.y = 0.2;
	r_pose.position.z = 0.2;
	r_pose.orientation.x = 0.577;
	r_pose.orientation.y = 0.577;
	r_pose.orientation.z = 0.577;
	r_pose.orientation.w = 1.0;
	
	left_hand.setPoseTarget(l_pose, "left_hand_eef");
	right_hand.setPoseTarget(r_pose, "right_hand_eef");

	left_hand.move();
	right_hand.move();
	
	ros::waitForShutdown();
}
