#include <ur5_kendama_control/ball_track_controller.hpp>

#include <std_msgs/Float64.h>

#include <vector>

ball_track_controller::ball_track_controller( ros::NodeHandle& nh ) 
    : nh(nh),
    ball_pos_sub(nh),
    robotHeight(0.35),
    catchHeight(0.0),
    firstJntState(true),
    heightInitialized(false),
    descending(false)
{
    std::cout << "ball_track_controller::ball_track_controller" << std::endl;

    sub_js = nh.subscribe("/joint_states", 10, &ball_track_controller::jointStateCallback, this);

    joint_state.resize(6);
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = 0.0;
    }
}

void ball_track_controller::initializeParameters( RMLPositionInputParameters** ip, KDL::ChainFkSolverPos** fk_pos, 
        KDL::ChainIkSolverPos** ik_pos, KDL::ChainIkSolverVel** ik_vel )
{   
    ip_ref = ip;
    fk_pos_ref = fk_pos;
    ik_pos_ref = ik_pos;
    ik_vel_ref = ik_vel;
}


void ball_track_controller::updateHook() 
{
    // Get updated ball position
    std::vector<float> ballPos = ball_pos_sub.getBallPos();
    double ballVertVelo = ball_pos_sub.getBallVelo(); 

    // If ball position is updated, move the robot to track the ball
    if ( !(ballPos.at(0) == 0.0 && ballPos.at(1) == 0.0 && ballPos.at(2) == 0.0) )
    {
        trackBall(ballPos.at(0), ballPos.at(1), ballPos.at(2), ballVertVelo);
    }

    /*KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;

    KDL::ChainFkSolverPos* fk_pos = *(fk_pos_ref);
    fk_pos->JntToCart(q_cur, p_cur, -1);

    double roll, pitch, yaw;
    p_cur.M.GetRPY(roll, pitch, yaw);

    std::cout << "Ball pos: " << ballPos.at(0) << ", " << ballPos.at(1) << ", " << ballPos.at(2) << std::endl;
    if ( abs(ballVertVelo) > 1e-7 )
    {
        std::cout << "Ball vert velo: " << ballVertVelo << std::endl;
    }
    
    std::cout << "Arm pos: " << p_cur.p.x() << ", " << p_cur.p.y() <<  ", " << p_cur.p.z() << std::endl;
    std::cout << "Arm rpy: " << roll << ", " << pitch << ", " << yaw << std::endl << std::endl;*/
}

void ball_track_controller::setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot )
{   
    // extract out components of the input joint array to set as target positions
    for ( int i = 0; i < 6; i++ )
    {
        RMLPositionInputParameters* ip = *(ip_ref);

        // Set velo/accel limits
        ip->MaxVelocityVector->VecData[i] = 10;
        ip->MaxAccelerationVector->VecData[i] = 50.0;
        ip->MaxJerkVector->VecData[i] = 100.0;

        // Set position/velo targets
        ip->TargetPositionVector->VecData[i] = q.data[i];
        ip->TargetVelocityVector->VecData[i] = q_dot.data[i];
        
    }
}


void ball_track_controller::jointStateCallback( const sensor_msgs::JointState& js )
{
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = js.position[i];
    }

    float future0 = joint_state.data[2];
    float future2 = joint_state.data[0];

    joint_state.data[0] = future0;
    joint_state.data[2] = future2;

    firstJntState = false;
}

void ball_track_controller::initializeHeight( float z, KDL::Rotation orientation )
{
    robotHeight = z;
    catchHeight = z;
    robotOrientation = orientation;

    heightInitialized = true;

    descending = false;

    std::cout << "Height Initialized: " << robotHeight << std::endl;
}


void ball_track_controller::resetTrack()
{
    heightInitialized = false;
}


void ball_track_controller::trackBall(float ball_x, float ball_y, float ball_z, double ballVertVelo)
{
    // Get current cartesian position of end effector
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    KDL::ChainFkSolverPos* fk_pos = *(fk_pos_ref);
    fk_pos->JntToCart(q_cur, p_cur, -1);
    float cupHeight = p_cur.p.z();

    bool simulation;
    nh.param("/simulation", simulation, bool());

    float distToCup = ball_z - cupHeight;
    distToCup = simulation ?  distToCup - 0.13 : distToCup - 0.03;

    // If ball is above robot and ball is sufficiently above the table
    if ( !firstJntState && heightInitialized && distToCup >= 0.0 && ball_z >= 0.1)
    {
        // Compute  position of being underneath the ball
        KDL::Frame p_desired = p_cur;

        // If ball is on its way up or significantly above robot, only track x-y pos
        if ( distToCup > 0.1 || (ballVertVelo > 0 && !descending) )
        {
            KDL::Vector ball_pos(ball_x, ball_y, robotHeight);
            p_desired.p = ball_pos;
            p_desired.M = robotOrientation;

            // Compute inverse kinematics to ball x-y position
            KDL::JntArray q_desired(6);
            KDL::ChainIkSolverPos* ik_pos = *(ik_pos_ref);
            ik_pos->CartToJnt(q_cur, p_desired, q_desired);

            // Set zero desired joint velo
            KDL::JntArray zero_vel(6);
            zero_vel.data[0] = 0.0;
            zero_vel.data[1] = 0.0;
            zero_vel.data[2] = 0.0;
            zero_vel.data[3] = 0.0;
            zero_vel.data[4] = 0.0;
            zero_vel.data[5] = 0.0;

            // Send command for mid-waypoint
            setJointPos(q_desired, zero_vel);
        }
        
        // else if ball is approaching kendama cup, start catch procedure
        else if ( ballVertVelo < 0 && distToCup > 0.05 )
        {            
            descending = true;
            
            // take 90% of downward ball velo
            double dv = ballVertVelo/3;
            double desired_z = 0.1;

            catchHeight = p_cur.p.z();

            // set desired pose of end effector
            KDL::Vector ball_pos(ball_x, ball_y, desired_z);
            p_desired.p = ball_pos;
            p_desired.M = robotOrientation;

            // Get desired downward joint state from IK
            KDL::JntArray q_desired(6);
            KDL::ChainIkSolverPos* ik_pos = *(ik_pos_ref);
            ik_pos->CartToJnt(q_cur, p_desired, q_desired);

            // Set mid-waypoint twist velo
            KDL::Vector rot(0.0, 0.0, 0.0);
            KDL::Vector vel(0.0, 0.0, dv);
            KDL::Twist v_desired;
            v_desired.rot = rot;
            v_desired.vel = vel;

            // Get mid-waypoint joint velo from IK_vel
            KDL::JntArray q_dot_desired(6);
            KDL::ChainIkSolverVel* ik_vel = *(ik_vel_ref);
            ik_vel->CartToJnt(q_desired, v_desired, q_dot_desired);

            // Send command for mid-waypoint
            setJointPos(q_desired, q_dot_desired);
        }

        // else if ball is in cup but has not yet slowed down
        else if ( ballVertVelo < 0 && abs(ballVertVelo) > 0.005 )
        {
            double desired_z = catchHeight > 0.15 ? catchHeight - 0.1 : 0.05;

            // set desired pose of end effector
            KDL::Vector ball_pos(ball_x, ball_y, desired_z);
            p_desired.p = ball_pos;
            p_desired.M = robotOrientation;

            // Get desired downward joint state from IK
            KDL::JntArray q_desired(6);
            KDL::ChainIkSolverPos* ik_pos = *(ik_pos_ref);
            ik_pos->CartToJnt(q_cur, p_desired, q_desired);

            // Set zero desired joint velo
            KDL::JntArray zero_vel(6);
            zero_vel.data[0] = 0.0;
            zero_vel.data[1] = 0.0;
            zero_vel.data[2] = 0.0;
            zero_vel.data[3] = 0.0;
            zero_vel.data[4] = 0.0;
            zero_vel.data[5] = 0.0;

            // Send command for mid-waypoint
            setJointPos(q_desired, zero_vel);
        }
    }
}