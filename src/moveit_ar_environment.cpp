#include <ARWorldBuilder.h>

using namespace moveit;
using namespace std;
using namespace nxr;

int main(int argc, char **argv)
{
	ros::init( argc, argv, "nxr_alvar_blocks" );
	ros::NodeHandle nh;
		
	ros::Rate loop_rate(100);
	ARWorldBuilder block_world;

	// Updating the environment
	while( ros::ok() ) {
		block_world.updateWorld();
		
		loop_rate.sleep();
		nh.spinOnce();
	}

}
