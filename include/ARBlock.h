#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject>
#include <sstream>
#include <string>

namespace nxr {

class ARBlock {
public:
	
	ARBlock();
	ARBlock(float *dims, int id = 0);	
	moveit_msgs::CollisionObject toCollisionObject(std::string planning_frame = std::string("/base"));	

	geometry_msgs::Pose pose_;
	int id_;
	
	union dimensions_ {
		float all[3];
		struct {
			float x;
			float y;
			float z;
		};
	};
};

}
