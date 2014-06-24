#include <ar_block.h>
#include <sstream>
#include <string>

ar_block::ar_block()
{
	// Start off just testing Block types B
	dimensions.x = dimensions.y =	dimensions.z = 5.1;
	// position.x = position.xy = position.z = 0.0;
	// POSE DEFAULT VALUES ARE ALL ZERO
	pose.orientation.w = 1.0;
}

ar_block::ar_block(float *dims, int i) : id(i)
{
	dimensions.x = dims[0];
	dimensions.y = dims[1];
	dimensions.z = dims[2];
}

moveit_msgs::CollisionObject ar_block::toCollisionObject(std::string planning_frame)
{
    moveit_msgs::CollisionObject block;
    block.header.frame_id = planning_frame;
		std::sstream ss;
    ss << blocks.id;
    block.id = ss.str();

    vector < shape_msgs::SolidPrimitive > primitive_objects( 1 );
    vector < geometry_msgs::Pose > primitive_object_poses( 1 );
    primitive_objects[0].type = shape_msgs::SolidPrimitive::BOX;
    primitive_objects[0].dimensions.resize(3);
    primitive_objects[0].dimensions[0] = dimensions[0];
    primitive_objects[0].dimensions[1] = dimensions[1];
    primitive_objects[0].dimensions[2] = dimensions[2];
    block.primitives = primitive_objects;

    block.primitive_poses = primitive_object_poses;
		block.primitive_poses[0] = pose;
    block.operation = moveit_msgs::CollisionObject::ADD;
		
		return block;
}
