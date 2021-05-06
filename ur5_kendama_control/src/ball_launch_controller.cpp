#include <ur5_kendama_control/ball_launch_controller.hpp>

#include <std_msgs/Float64.h>

#include <kdl/frames.hpp>

ball_launch_controller::ball_launch_controller( ros::NodeHandle& nh ) 
    : nh(nh),
    launchFinished(true),
    launchFinishedFirstSegment(true),
    prevNorm(1000)
{
    std::cout << "ball_launch_controller::ball_launch_controller" << std::endl;

    sub_js = nh.subscribe("/joint_states", 10, &ball_launch_controller::jointStateCallback, this);

    joint_state.resize(6);
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = 0.0;
    }
}


void ball_launch_controller::initializeParameters( RMLPositionInputParameters** ip, KDL::ChainFkSolverPos** fk_pos, 
        KDL::ChainIkSolverPos** ik_pos, KDL::ChainIkSolverVel** ik_vel )
{   
    ip_ref = ip;
    fk_pos_ref = fk_pos;
    ik_pos_ref = ik_pos;
    ik_vel_ref = ik_vel;
}


void ball_launch_controller::updateHook() 
{
    // Compute distance to commanded position
    KDL::JntArray diff(6);
    KDL::Subtract(joint_state, q_commanded, diff);

    float sum_of_squares = diff.data[0]*diff.data[0] + diff.data[1]*diff.data[1] + 
                diff.data[2]*diff.data[2] + diff.data[3]*diff.data[3] + 
                diff.data[4]*diff.data[4] + diff.data[5]*diff.data[5];

    float norm_diff = pow(sum_of_squares, 0.5);

    // If commmanded position achieved, proceed to next segment of launch
    if( norm_diff < 0.1 || prevNorm < norm_diff )
    {
        // If just finished first segment of launch
        if ( !launchFinishedFirstSegment )
        {
            launchBallLastSegment();
            launchFinishedFirstSegment = true;
            prevNorm = 1000;
        }

        // Else, we just finished second segment of launch
        else
        {
            launchFinished = true;
            prevNorm = 1000;
        }
    } 
        
    // Else, we have not yet achieved the commanded position
    else 
    {
        prevNorm = norm_diff;
    }


    // Get current position from FK
    //KDL::JntArray q_cur = joint_state;
    //KDL::Frame p_cur;
    //KDL::ChainFkSolverPos* fk_pos = *(fk_pos_ref);
    //fk_pos->JntToCart(q_cur, p_cur, -1);
    // 0.817194, 0.303569
    //std::cout << p_cur.p.x() << ", " << p_cur.p.y() << ", " << p_cur.p.z() << std::endl; 
}


void ball_launch_controller::setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot )
{
    float max = 0.0;

    for ( int i = 0; i < 6; i++ )
    {
        if ( q_dot.data[i] > max )
        {
            max = q_dot.data[i];
        }
    }
    std::cout << "Max joint velo " << max << "rad/s" << std::endl;

    // extract out components of the input joint array to set as target positions
    for ( int i = 0; i < 6; i++ )
    {
        RMLPositionInputParameters* ip = *(ip_ref);
        
        // Set velo/accel limits
        ip->MaxVelocityVector->VecData[i] = 15;
        ip->MaxAccelerationVector->VecData[i] = 150.0;
        ip->MaxJerkVector->VecData[i] = 400.0;

        // Set position/velo targets
        ip->TargetPositionVector->VecData[i] = q.data[i];
        ip->TargetVelocityVector->VecData[i] = q_dot.data[i];
    }
}

void ball_launch_controller::jointStateCallback(const sensor_msgs::JointState& js )
{
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = js.position[i];
    }

    float future0 = joint_state.data[2];
    float future2 = joint_state.data[0];

    joint_state.data[0] = future0;
    joint_state.data[2] = future2;

}

void ball_launch_controller::launchBallFirstSegment()
{
    // Get current position from FK
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    KDL::ChainFkSolverPos* fk_pos = *(fk_pos_ref);
    fk_pos->JntToCart(q_cur, p_cur, -1);

    // Compute mid-waypoint position
    KDL::Frame p_mid_desired = p_cur;
    KDL::Vector mid(0.0, 0.0, 0.05);
    p_mid_desired.p += mid;

    // Get mid-waypoint joint state from IK
    KDL::JntArray q_mid_desired(6);
    KDL::ChainIkSolverPos* ik_pos = *(ik_pos_ref);
    ik_pos->CartToJnt(q_cur, p_mid_desired, q_mid_desired);

    // Set mid-waypoint twist velo
    KDL::Vector rot(0.0, 0.0, 0.0);
    KDL::Vector vel(0.0, 0.0, 3.5);
    KDL::Twist v_desired;
    v_desired.rot = rot;
    v_desired.vel = vel;

    // Get mid-waypoint joint velo from IK_vel
    KDL::JntArray q_dot_desired(6);
    KDL::ChainIkSolverVel* ik_vel = *(ik_vel_ref);
    ik_vel->CartToJnt(q_mid_desired, v_desired, q_dot_desired);

    launchFinished = false;
    launchFinishedFirstSegment = false;
    
    q_commanded = q_mid_desired;
    std::cout << "Sending midpoint command..." << std::endl;

    // Send command for mid-waypoint
    setJointPos(q_mid_desired, q_dot_desired);
}

void ball_launch_controller::launchBallLastSegment()
{
    // Get current position from FK
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    KDL::ChainFkSolverPos* fk_pos = *(fk_pos_ref);
    fk_pos->JntToCart(q_cur, p_cur, -1);

    // Compute end-waypoint position
    KDL::Frame p_end_desired = p_cur;
    KDL::Vector end(0.0, 0.0, 0.05);
    p_end_desired.p += end;

    // Get end-waypoint joint state from IK
    q_cur = joint_state;
    KDL::JntArray q_end_desired(6);
    KDL::ChainIkSolverPos* ik_pos = *(ik_pos_ref);
    ik_pos->CartToJnt(q_cur, p_end_desired, q_end_desired);

    // Set end-waypoint joint velo
    KDL::JntArray zero_vel(6);
    zero_vel.data[0] = 0.0;
    zero_vel.data[1] = 0.0;
    zero_vel.data[2] = 0.0;
    zero_vel.data[3] = 0.0;
    zero_vel.data[4] = 0.0;
    zero_vel.data[5] = 0.0;

    q_commanded = q_end_desired;
    std::cout << "Sending endpoint command..." << std::endl;

    // Send command for mid-waypoint
    setJointPos(q_end_desired, zero_vel);
}


bool ball_launch_controller::finishedLaunch()
{
    return launchFinished;
}