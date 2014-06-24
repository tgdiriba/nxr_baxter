<<<<<<< HEAD
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
#include <sstream>
#include <pthread.h>
#include <ar_block.h>


using namespace moveit;
using namespace std;

void setupCageEnvironment(string planning_frame = string("/base"))
{

	ros::NodeHandle nh;

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
	 
	for( vector< moveit_msgs::CollisionObject >::iterator object = object_collection.begin(); object != object_collection.end(); object++ ) {
		object_publisher.publish(*object);
	}
	
	ros::Duration(2.0).sleep();
}

void updateBlocks( vector< ar_block > blocks )
{
	ros::NodeHandle nh;
	ros::Publisher block_publisher = nh.advertise< moveit_msgs::CollisionObject >("collision_object", 10);
	vector< moveit_msgs::CollisionObject > block_collection( blocks.size() );
	
	for( vector< ar_block >::iterator it = blocks.begin(); it != blocks.end(); it++ ) {
		
		block_publisher.publisher( it.toCollisionObject() );

	}

	// Wait for all of the blocks to be registered
	ros::Duration(2.0).sleep();	
}
=======
#include <nxr_baxter/ARWorldBuilder.h>

using namespace moveit;
using namespace std;
using namespace nxr;
>>>>>>> e0bf515741c6248b2f690e60c3ddc611a2a29b09

int main(int argc, char **argv)
{
	ros::init( argc, argv, "nxr_alvar_blocks" );
	ros::NodeHandle nh;
		
<<<<<<< HEAD
	ros::AsyncSpinner spinner(1);
	spinner.start();

	planning_interface::PlanningSceneInterface scene;
	move_group_interface::MoveGroup both_arms("both_arms");
	move_group_interface::MoveGroup left_arm("left_arm");
	move_group_interface::MoveGroup right_arm("right_arm");

	setupCageEnvironment();

	// Getting the PlanningScene
	ros::ServiceClient servc = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

	moveit_msgs::GetPlanningScene get_ps;
	get_ps.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
=======
	ros::Rate loop_rate(30);
	ARWorldBuilder block_world;
	ROS_INFO("Successfully built AR World...");
>>>>>>> e0bf515741c6248b2f690e60c3ddc611a2a29b09
	
	// Updating the environment
	while( ros::ok() ) {
		block_world.updateWorld();
		
		loop_rate.sleep();
		ros::spinOnce();
	}

}
