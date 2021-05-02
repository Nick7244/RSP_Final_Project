#include <test_ball_pos/ball_position_publisher.hpp> 

int main( int argc, char** argv )
{
    // Initialize ROS node and create node handle
    ros::init(argc, argv, "ballPosPubNode");
    ros::NodeHandle nh_pub;

    // Pass node handle to publisher class to create publisher
    ball_pos_publisher pub(nh_pub);

	// Rate object used to specify loops at specific frequency
	// The input to this is frequency in Hz
	ros::spin();

    return 0;
}