#ifndef ARBLOCK_H
#define ARBLOCK_H

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <sstream>
#include <string>

namespace nxr {

typedef struct _dimensions {
	float x;
	float y;
	float z;
} dimensions;

class ARBlock {
public:
	
	ARBlock();
	ARBlock(float *dims, int id = 0);	
	moveit_msgs::CollisionObject toCollisionObject(std::string planning_frame = std::string("/base"));	

	geometry_msgs::Pose pose_;
	int id_;
	
	union {
		float dims_[3];
		dimensions dimensions_;
	};
};

}

#endif
