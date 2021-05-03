#include <ur5_kendama_track_ball_controller/ball_track_controller.hpp>

#include <std_msgs/Float64.h>

#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <vector>

#include <rtt/Component.hpp> // This should come last in the include list


ball_track_controller::ball_track_controller( const std::string& name ) 
    : TaskContext(name),
    port_msr_jnt_state("Measured joint state"),
    ball_pos_sub(nh),
    robotHeight(0.0),
    firstJntState(true)
{
    std::cout << "ball_track_controller::ball_track_controller" << std::endl;

    std::string robot_description_string;
    nh.param("/robot_description", robot_description_string, std::string());

    sub_js = nh.subscribe("/joint_states", 10, &ball_track_controller::jointStateCallback, this);

    // Obtain the IK solver
    if ( kdl_parser::treeFromString(robot_description_string, tree) )
    {
        if ( tree.getChain("base_link", "ee_link", chain) )
        {
            fk_pos = new KDL::ChainFkSolverPos_recursive(chain);
            ik_vel = new KDL::ChainIkSolverVel_pinv(chain);
            ik_pos = new KDL::ChainIkSolverPos_NR(chain, *fk_pos, *ik_vel);
        }

        else 
        {
            ROS_ERROR("Cannot get a chain");
        }
    }

    else
    {
        ROS_ERROR("Cannot get a tree");
    }

    joint_state.resize(6);
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = 0.0;
    }

    addPort("MsrJntState", port_msr_jnt_state);

    addOperation("GetJointPos", &ball_track_controller::getJointPos, this, RTT::OwnThread); 
    addOperation("SetJointPos", &ball_track_controller::setJointPos, this, RTT::OwnThread);
}


bool ball_track_controller::configureHook() 
{
    std::cout << "ball_track_controller::configureHook" << std::endl;

    // initialize trajectory generation objects
    rml = new ReflexxesAPI(6, getPeriod()); // gets period from task context object
    ip = new RMLPositionInputParameters(6);
    op = new RMLPositionOutputParameters(6);

    // initialize trajectory generation parameters
    for ( int i = 0; i < 6; i++ )
    {
        ip->CurrentPositionVector->VecData[i] = 0.0;
        ip->CurrentVelocityVector->VecData[i] = 0.0;
        ip->CurrentAccelerationVector->VecData[i] = 0.0;

        ip->MaxAccelerationVector->VecData[i] = 100.0;
        ip->MaxJerkVector->VecData[i] = 500.0;

        ip->SelectionVector->VecData[i] = true;
    }

    ip->MaxVelocityVector->VecData[0] = 3.15;
    ip->MaxVelocityVector->VecData[1] = 3.15;
    ip->MaxVelocityVector->VecData[2] = 3.15;
    ip->MaxVelocityVector->VecData[3] = 3.20;
    ip->MaxVelocityVector->VecData[4] = 3.20;
    ip->MaxVelocityVector->VecData[5] = 3.20;

    ip->TargetPositionVector->VecData[0] = 0.0;
    ip->TargetVelocityVector->VecData[0] = 0.0;
    
    ip->TargetPositionVector->VecData[1] = -1.5708;
    ip->TargetVelocityVector->VecData[1] = 0.0;
    
    ip->TargetPositionVector->VecData[2] = 1.8708;
    ip->TargetVelocityVector->VecData[2] = 0.0;
    
    ip->TargetPositionVector->VecData[3] = -0.3;
    ip->TargetVelocityVector->VecData[3] = 0.0;
    
    ip->TargetPositionVector->VecData[4] = 1.5708;
    ip->TargetVelocityVector->VecData[4] = 0.0;
    
    ip->TargetPositionVector->VecData[5] = 0.0;
    ip->TargetVelocityVector->VecData[5] = 0.0;
}


bool ball_track_controller::startHook() 
{
    std::cout << "ball_track_controller::startHook" << std::endl;
}


void ball_track_controller::updateHook() 
{
    // Get updated ball position
    std::vector<float> ballPos = ball_pos_sub.getBallPos();

    // If ball position is updated, move the robot to track the ball
    if ( !(ballPos.at(0) == 0.0 && ballPos.at(1) == 0.0 && ballPos.at(2) == 0.0) )
    {
        trackBall(ballPos.at(0), ballPos.at(1));

        // compute an iteration of the trajectory
        int result = rml->RMLPosition(*ip, op, flags);

        // setup for next iteration
        *ip->CurrentPositionVector = *op->NewPositionVector;
        *ip->CurrentVelocityVector = *op->NewVelocityVector;
        *ip->CurrentAccelerationVector = *op->NewAccelerationVector;

        // get the joint values and write them to the port
        std_msgs::Float64MultiArray js;
        js.data.resize(6);
        
        for ( int i = 0; i < 6; i++ )
        {
            op->GetNewPositionVectorElement(&js.data[i], i);
        }

        port_msr_jnt_state.write(js);
    }

    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    fk_pos->JntToCart(q_cur, p_cur, -1);

    double roll, pitch, yaw;
    p_cur.M.GetRPY(roll, pitch, yaw);

    std::cout << "Ball pos: " << ballPos.at(0) << ", " << ballPos.at(1) << ", " << ballPos.at(2) << std::endl;
    std::cout << "Arm pos: " << p_cur.p.x() << ", " << p_cur.p.y() <<  ", " << p_cur.p.z() << std::endl;
    std::cout << "Arm rpy: " << roll << ", " << pitch << ", " << yaw << std::endl << std::endl;
}


void ball_track_controller::stopHook() 
{
    std::cout << "ball_track_controller::stopHoop" << std::endl;
}


void ball_track_controller::cleanupHook() 
{
    std::cout << "ball_track_controller::cleanupHook" << std::endl;
}


KDL::JntArray ball_track_controller::getJointPos()
{
    // return the current joint position
    KDL::JntArray js(6);
    for ( int i = 0; i < 6; i++ )
    {
        js.data[i] = ip->CurrentPositionVector->VecData[i];
    }

    return js;
}


void ball_track_controller::setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot )
{
    // extract out components of the input joint array to set as target positions
    for ( int i = 0; i < 6; i++ )
    {
        ip->TargetPositionVector->VecData[i] = q.data[i];
        ip->TargetVelocityVector->VecData[i] = q_dot.data[i];
    }
}


void ball_track_controller::jointStateCallback(const sensor_msgs::JointState& js )
{
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = js.position[i];
    }

    float future0 = joint_state.data[2];
    float future2 = joint_state.data[0];

    joint_state.data[0] = future0;
    joint_state.data[2] = future2;

    if ( firstJntState )
    {
        KDL::JntArray q_cur = joint_state;
        KDL::Frame p_cur;
        fk_pos->JntToCart(q_cur, p_cur, -1);

        robotHeight = p_cur.p.z();
        robotOrientation = p_cur.M;

        firstJntState = false;

        std::cout << "Initialized height: " << robotHeight << std::endl;
    }

}


void ball_track_controller::trackBall(float ball_x, float ball_y)
{
    if ( !firstJntState )
    {
        // Get current cartesian position
        KDL::JntArray q_cur = joint_state;
        KDL::Frame p_cur;
        fk_pos->JntToCart(q_cur, p_cur, -1);

        // Compute  position of being underneath the ball
        KDL::Frame p_desired = p_cur;
        KDL::Vector ball_pos(ball_x, ball_y, robotHeight);
        p_desired.p = ball_pos;
        p_desired.M = robotOrientation;

        // Compute inverse kinematics to ball x-y position
        KDL::JntArray q_desired(6);
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


ORO_CREATE_COMPONENT(ball_track_controller) // register the RTT component