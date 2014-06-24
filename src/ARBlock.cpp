#include <ARBlock.h>

namespace nxr {

ARBlock::ARBlock()
{
	// Start off just testing Block types B
	dimensions_.x = dimensions_.y =	dimensions_.z = 5.1;
	// position.x = position.xy = position.z = 0.0;
	// POSE DEFAULT VALUES ARE ALL ZERO
	pose_.orientation.w = 1.0;
}

ARBlock::ARBlock(float *dims, int i) : id(i)
{
	dimensions_.x = dims[0];
	dimensions_.y = dims[1];
	dimensions_.z = dims[2];
}

moveit_msgs::CollisionObject ARBlock::toCollisionObject(std::string planning_frame)
{
    moveit_msgs::CollisionObject block;
    block.header.frame_id = planning_frame;
		std::sstream ss;
    ss << id_;
    block.id = ss.str();

    vector < shape_msgs::SolidPrimitive > primitive_objects( 1 );
    vector < geometry_msgs::Pose > primitive_object_poses( 1 );
    primitive_objects[0].type = shape_msgs::SolidPrimitive::BOX;
    primitive_objects[0].dimensions.resize(3);
    primitive_objects[0].dimensions[0] = dimensions_[0];
    primitive_objects[0].dimensions[1] = dimensions_[1];
    primitive_objects[0].dimensions[2] = dimensions_[2];
    block.primitives = primitive_objects;

    block.primitive_poses = primitive_object_poses;
		block.primitive_poses[0] = pose_;
    block.operation = moveit_msgs::CollisionObject::ADD;
		
		return block;
}

}
