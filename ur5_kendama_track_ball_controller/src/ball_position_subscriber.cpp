#include <ur5_kendama_track_ball_controller/ball_position_subscriber.hpp> 

ball_pos_subscriber::ball_pos_subscriber( ros::NodeHandle& nh )
    : nh(nh)
{
    // node_handle.subscribe(topic_name, queue_size, function_pointer, obj) returns a subscriber 
    // that is subscribed to <topic_name>, and calls <function_pointer> on the object <obj> whenever 
    // a message has arrived
    sub = nh.subscribe("ball_position", 10, &ball_pos_subscriber::callback, this);

    for( int i = 0; i < 3; i++ )
    {
        ball_pos.push_back(0.0);
    }
}

ball_pos_subscriber::~ball_pos_subscriber() {}

// Function that automatically gets called whenever new data comes in
void ball_pos_subscriber::callback( const ur5_kendama_msgs::ball_position& msg )
{
    ball_pos[0] = msg.x.data;
    ball_pos[1] = msg.y.data;
    ball_pos[2] = msg.z.data;
}

std::vector<float> ball_pos_subscriber::getBallPos()
{
    return ball_pos;
}