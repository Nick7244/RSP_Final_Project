#include <test_ball_pos/ball_position_publisher.hpp> 

ball_pos_publisher::ball_pos_publisher( ros::NodeHandle& nh )
    : nh(nh)
{
    // node_handle.advertise<data_type>(topic_name, queue_size) returns a publisher that 
	// advertises the <data_type>, <topic_name> is the name of the topic that is 
	// advertised, <queue_size> = 10 is fine
    pub = nh.advertise<ur5_kendama_msgs::ball_position>("ball_position", 10);

    deployerSub = nh.subscribe("ball_position_deployer", 10, &ball_pos_publisher::callback, this);
}

ball_pos_publisher::~ball_pos_publisher() {}

// Function that calls the publisher object to publish the data
void ball_pos_publisher::publish() 
{
    // Publish the custom message
    pub.publish(myMessage);
}

void ball_pos_publisher::callback( const std_msgs::Float64MultiArray& ball_pos_ary )
{
    // Create the custom message
    myMessage.x.data = ball_pos_ary.data[0];
    myMessage.y.data = ball_pos_ary.data[1];
    myMessage.z.data = ball_pos_ary.data[2];

    publish();    
}