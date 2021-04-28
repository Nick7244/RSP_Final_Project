#include <test_ball_pos/ball_position_publisher.hpp> 
#include <ur5_kendama_msgs/ball_position.h>

ball_pos_publisher::ball_pos_publisher( ros::NodeHandle& nh )
    : nh(nh)
{
    // node_handle.advertise<data_type>(topic_name, queue_size) returns a publisher that 
	// advertises the <data_type>, <topic_name> is the name of the topic that is 
	// advertised, <queue_size> = 10 is fine
    pub = nh.advertise<ur5_kendama_msgs::ball_position>("ball_position", 10);
}

ball_pos_publisher::~ball_pos_publisher() {}

// Function that calls the publisher object to publish the data
void ball_pos_publisher::publish() 
{
    // Create the custom message
    ur5_kendama_msgs::ball_position myMessage;
    myMessage.x.data = 14;
    myMessage.y.data = 7;
    myMessage.z.data = 2;

    // Publish the custom message
    pub.publish(myMessage);
}