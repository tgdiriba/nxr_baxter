#include <nxr_baxter/ARWorldBuilder.h>

using namespace moveit;
using namespace std;
using namespace nxr;

int main(int argc, char **argv)
{
	ros::init( argc, argv, "nxr_alvar_blocks" );
	ros::NodeHandle nh;
		
	ros::Rate loop_rate(30);
	ARWorldBuilder block_world;
	ROS_INFO("Successfully built AR World...");
	
	// Updating the environment
	while( ros::ok() ) {
		block_world.updateWorld();
		
		loop_rate.sleep();
		ros::spinOnce();
	}

}
