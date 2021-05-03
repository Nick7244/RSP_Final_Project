#include <ball_plugin/ball_sub.hpp> 

int main( int argc, char** argv )
{
    // Initialize ROS node and create node handle
    ros::init(argc, argv, "ballPosSubNode");
    ros::NodeHandle nh_sub;

    // Pass node handle to subscriber class to create subscriber
    ball_pos_subscriber sub(nh_sub);

    // ROS command to pause the program, preventing it from exiting,
	// allowing the subscriber to do its job
    ros::spin();

    return 0;
}