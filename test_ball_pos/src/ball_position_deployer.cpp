#include <test_ball_pos/ball_position_deployer.hpp>

#include <rtt/Component.hpp> // This should come last in the include list

ball_pos_deployer::ball_pos_deployer( const std::string& name ) 
    : TaskContext(name),
    port_ball_pos("Ball position")
{
    ball_pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
    ball_pos.layout.dim[0].label = std::string("ball_pos");
    ball_pos.layout.dim[0].size = 3;
    ball_pos.layout.dim[0].stride = 1;
    ball_pos.layout.data_offset = 0;

    ball_pos.data.push_back(0.0);
    ball_pos.data.push_back(0.0);
    ball_pos.data.push_back(0.0);

    addPort("BallPosition", port_ball_pos);

    addOperation("GetBallPos", &ball_pos_deployer::getBallPos, this, RTT::OwnThread); 
    addOperation("SetBallPos", &ball_pos_deployer::setBallPos, this, RTT::OwnThread);
    addOperation("PublishBallPos", &ball_pos_deployer::publish, this, RTT::OwnThread);
}


ball_pos_deployer::~ball_pos_deployer() {}


void ball_pos_deployer::publish() 
{
    port_ball_pos.write(ball_pos);
}


KDL::JntArray ball_pos_deployer::getBallPos()
{
    KDL::JntArray ball_pos_ary(3);

    ball_pos_ary.data[0] = ball_pos.data[0];
    ball_pos_ary.data[1] = ball_pos.data[1];
    ball_pos_ary.data[2] = ball_pos.data[2];

    return ball_pos_ary;
}


void ball_pos_deployer::setBallPos( KDL::JntArray ball_pos_ary )
{
    ball_pos.data[0] = ball_pos_ary.data[0];
    ball_pos.data[1] = ball_pos_ary.data[1];
    ball_pos.data[2] = ball_pos_ary.data[2];
}


bool ball_pos_deployer::configureHook()
{
    std::cout << "ball_track_controller::configureHook" << std::endl;
}


bool ball_pos_deployer::startHook()
{
    std::cout << "ball_track_controller::startHook" << std::endl;
}


void ball_pos_deployer::updateHook()
{

}


void ball_pos_deployer::stopHook()
{
    std::cout << "ball_track_controller::stopHoop" << std::endl;
}


void ball_pos_deployer::cleanupHook()
{
    std::cout << "ball_track_controller::cleanupHook" << std::endl;
}


ORO_CREATE_COMPONENT(ball_pos_deployer) // register the RTT component