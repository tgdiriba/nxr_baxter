#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <std_msgs/Header.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <string>
#include <pthread.h>

using namespace moveit;
using namespace std;

void setupCageEnvironment(ros::NodeHandle& nh, string planning_frame = string("/base"))
{

	ros::Publisher object_publisher = nh.advertise< moveit_msgs::CollisionObject >("collision_object",10);

	vector< moveit_msgs::CollisionObject > object_collection(1);

	// Setup the table
	object_collection[0] = moveit_msgs::CollisionObject();
	object_collection[0].header.frame_id = planning_frame;	
	object_collection[0].id = "table";
	
	vector < shape_msgs::SolidPrimitive > primitive_objects(1);
	primitive_objects[0].type = shape_msgs::SolidPrimitive::BOX;
	primitive_objects[0].dimensions.resize(3); 
	primitive_objects[0].dimensions[0] = 0.75; 
	primitive_objects[0].dimensions[1] = 1.5; 
	primitive_objects[0].dimensions[2] = 0.375; 
	object_collection[0].primitives = primitive_objects;

	vector < geometry_msgs::Pose > primitive_object_poses(1);
	primitive_object_poses[0].position.x = 1.0;
	primitive_object_poses[0].position.y = 0.0;	
	primitive_object_poses[0].position.z = -0.1875;
	primitive_object_poses[0].orientation.w = 1.0;
	object_collection[0].primitive_poses = primitive_object_poses;	
	object_collection[0].operation = moveit_msgs::CollisionObject::ADD;
	
	// Setup the walls
	

	// Setup the deskspace
	 
	// 
	for( vector< moveit_msgs::CollisionObject >::iterator object = object_collection.begin(); object != object_collection.end(); object++ ) {
		object_publisher.publish(*object);
	}
	
	ros::Duration(2.0).sleep();
}

int main(int argc, char **argv)
{
	ros::init( argc, argv, "moveit_test_3" );
	ros::NodeHandle nh;
		
	ros::AsyncSpinner spinner(1);
	spinner.start();

	planning_interface::PlanningSceneInterface scene;
	move_group_interface::MoveGroup both_arms("both_arms");
	move_group_interface::MoveGroup left_arm("left_arm");
	move_group_interface::MoveGroup right_arm("right_arm");

	setupCageEnvironment(nh);

	// Getting the PlanningScene
	ros::ServiceClient servc = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

	moveit_msgs::GetPlanningScene get_ps;
	get_ps.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
	
	if( !servc.call(get_ps) ) {
		ROS_ERROR("Failed to call service /get_planning_scene");
	}
	else {
		for( vector< moveit_msgs::CollisionObject >::iterator object = get_ps.response.scene.world.collision_objects.begin(); object != get_ps.response.scene.world.collision_objects.end(); object++ ) {
			ROS_INFO("Collision object id: %s", object->id.c_str());
		}
	}

	// Plan a motion with left_arm
	geometry_msgs::Pose left_target;
	left_target.position.x = 1.0;
	left_target.position.y = -0.7;
	left_target.position.z = 1.0;
	left_target.orientation.w = 1.0;

	left_arm.setPoseTarget(left_target);
	planning_interface::MoveGroup::Plan left_plan;
	bool success = left_arm.plan(left_plan);

	ROS_INFO("Left plan %s", success ? "success" : "failure");	

	if( success ) left_arm.move();

	ros::waitForShutdown();
}
