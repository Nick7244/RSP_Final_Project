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
	ros::Rate rate(1);

	// While the node handle is still OK
	// This only changes to alse once ros::shutdown() gets called
	while( nh_pub.ok() ) {
		// Sleeps for any time leftover in a cycle (i.e. 1 second in this case)
		// calculated from the last time sleep, reset or counstructor was called
		// i.e. this allows for running the while loop at the desired frequency
		rate.sleep();

		// Call method to publish the info
		pub.publish();
	}	

    return 0;
}