#include <ARWorldBuilder.h>

namespace nxr {

using namespace moveit;
using namespace std;

ARWorldBuilder::ARWorldBuilder(unsigned int cutoff) : cutoff_confidence_(cutoff)
{
	collision_object_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	ar_pose_marker_sub_ = nh_.subscribe("ar_pose_marker", 100, &ARWorldBuilder::arPoseMarkerCallback, this);
	setupCageEnvironment();
}

void ARWorldBuilder::arPoseMarkerCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers_msg)
{
	vector< ar_track_alvar::AlvarMarker >::iterator it = markers_msg->markers.begin();
	vector< ar_track_alvar::AlvarMarker >::iterator end = markers_msg->markers.end();
	
	for( ; it != end; it++) {
		// Use a cutoff confidence
		if( it->confidence >= cutoff_confidence_ ) {
			// Eventually differentiate the different marker types
			ar_blocks_[ it->id ].pose_ = it->pose.pose;
		}
	}
}

void ARWorldBuilder::setupCageEnvironment(string planning_frame)
{
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
		collision_object_pub_.publish(*object);
	}
	
	ros::Duration(2.0).sleep();
}

void ARWorldBuilder::updateWorld()
{
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	

	for( ; it != end; it++ ) {
		block_publisher.publisher( it->toCollisionObject() );
	}

	// Wait for all of the blocks to be registered
	ros::Duration(2.0).sleep();	
}

}
