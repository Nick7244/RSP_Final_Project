#include <ball_position_subscriber/ball_position_subscriber.hpp> 

ball_pos_subscriber::ball_pos_subscriber( ros::NodeHandle& nh )
    : nh(nh),
    timeLastBallPos(0.0),
    verticalVelo(0.0),
    firstBallPos(true),
    lastBallHeight(0.0)
{
    // node_handle.subscribe(topic_name, queue_size, function_pointer, obj) returns a subscriber 
    // that is subscribed to <topic_name>, and calls <function_pointer> on the object <obj> whenever 
    // a message has arrived
    sub = nh.subscribe("/ball_position", 10, &ball_pos_subscriber::callback, this);

    for( int i = 0; i < 3; i++ )
    {
        ball_pos.push_back(0.0);
    }
}

ball_pos_subscriber::~ball_pos_subscriber() {}

// Function that automatically gets called whenever new data comes in
void ball_pos_subscriber::callback( const ur5_kendama_msgs::ball_position& msg )
{
    // Get the current time
    double curTime = ros::Time::now().toSec();
    
    // If first ball position received, record the time
    if ( firstBallPos )
    {
        timeLastBallPos = curTime;
        lastBallHeight = msg.z.data;
        firstBallPos = false;
    }

    // If at least second ball position received, compute instantaneous velo
    else if (curTime - timeLastBallPos >= 0.015)
    {
        double dz = msg.z.data - lastBallHeight;
        double dt = curTime - timeLastBallPos;
        verticalVelo = dz/dt;

        timeLastBallPos = curTime;
        lastBallHeight = msg.z.data;
    }

    // Update internal state for ball position
    ball_pos[0] = msg.x.data;
    ball_pos[1] = msg.y.data;
    ball_pos[2] = msg.z.data;
}

std::vector<float> ball_pos_subscriber::getBallPos()
{
    return ball_pos;
}

double ball_pos_subscriber::getBallVelo()
{
    return verticalVelo;
}