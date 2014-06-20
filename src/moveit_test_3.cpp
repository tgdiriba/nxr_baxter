#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Header.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <string>
#include <pthread.h>

using namespace moveit;
using namespace std;

int main(int argc, char **argv)
{
	ros::init( argc, argv, "moveit_test_3" );
	
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup both_arms("both_arms");
	planning_interface::PlanningSceneInterface scene;

	vector< moveit_msgs::CollisionObject > object_collection;
	object_collection[0] = moveit_msgs::CollisionObject();
	object_collection[0].header = std_msgs::Header();	
	object_collection[0].id = string("table");
	vector < shape_msgs::SolidPrimitive > primitive_objects(1);
	primitive_objects[0].type = shape_msgs::SolidPrimitive::BOX;
	vector < double > dims(4);
	dims[0] = 1.5;
	dims[1] = 0.75;
	dims[2] = 0.75;
	
	primitive_objects[0].dimensions = dims; 
	object_collection[0].primitives = primitive_objects;

	vector < geometry_msgs::Pose > primitive_object_poses(1);
	primitive_object_poses[0].position.x = 0.75;
	primitive_object_poses[0].position.y = 0.0;	
	primitive_object_poses[0].position.z = 0.0;
	primitive_object_poses[0].orientation.w = 1.0;
	object_collection[0].primitive_poses = primitive_object_poses;	
	object_collection[0].operation = moveit_msgs::CollisionObject::ADD;

	// scene.addCollisionObjects(object_collection);
	// 	addCollisionObjects is not supported in Groovy!
	both_arms.setRandomTarget();
	both_arms.move();

	ros::waitForShutdown();
}
