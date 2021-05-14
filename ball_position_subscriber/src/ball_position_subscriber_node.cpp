#include <ball_position_subscriber/ball_position_subscriber.hpp> 
#include <ros/ros.h>

int main( int argc, char** argv )
{
    // Initialize ROS node and create node handle
    ros::init(argc, argv, "ballPosSubNode");
    ros::NodeHandle nh_sub;

    // Pass node handle to subscriber class to create subscriber
    ball_pos_subscriber sub(nh_sub);

    ros::spinOnce();

    // ROS command to pause the program, preventing it from exiting,
	// allowing the subscriber to do its job
    while ( ros::ok() )
    {
        std::vector<float> ball_pos = sub.getBallPos();
        std::cout << "Ball Position: x = " << ball_pos.at(0) << ", y = " << 
            ball_pos.at(1) << ", z = " << ball_pos.at(2) << std::endl;

        ros::spinOnce();
    }
    

    return 0;
}