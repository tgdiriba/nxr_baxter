#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject>

class ar_block {
public:
	ar_block();
	ar_block(float *dims, int id = 0);	
	moveit_msgs::CollisionObject toCollisionObject(std::string planning_frame = std::string("/base"));	

	geometry_msgs::Pose pose;
	int id;
	
	union {
		float all[3];
		struct {
			float x;
			float y;
			float z;
		};
	};
};
