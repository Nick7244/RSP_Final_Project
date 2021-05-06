#include <ur5_kendama_control/ur5_kendama_controller.hpp>

#include <std_msgs/Float64.h>

#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

#include <rtt/Component.hpp> // This should come last in the include list

ur5_kendama_controller::ur5_kendama_controller( const std::string& name ) 
    : TaskContext(name),
    port_msr_jnt_state("Measured joint state"),
    launchCommanded(false),
    prevNorm(1000)
{
    std::cout << "ur5_kendama_controller::ur5_kendama_controller" << std::endl;

    std::string robot_description_string;
    nh.param("/robot_description", robot_description_string, std::string());

    std::string end_effector_string;
    nh.param("/end_effector_name", end_effector_string, std::string());

    sub_js = nh.subscribe("/joint_states", 10, &ur5_kendama_controller::jointStateCallback, this);

    // Obtain the IK solver
    if ( kdl_parser::treeFromString(robot_description_string, tree) )
    {
        if ( tree.getChain("base_link", end_effector_string, chain) )
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

    addOperation("GetJointPos", &ur5_kendama_controller::getJointPos, this, RTT::OwnThread); 
    addOperation("SetJointPos", &ur5_kendama_controller::setJointPos, this, RTT::OwnThread);
    addOperation("ZeroPose", &ur5_kendama_controller::commandZeroPose, this, RTT::OwnThread);
    addOperation("TPose", &ur5_kendama_controller::commandTPose, this, RTT::OwnThread);
    addOperation("LaunchBall", &ur5_kendama_controller::launchBallFirstSegment, this, RTT::OwnThread);
}


bool ur5_kendama_controller::configureHook() 
{
    std::cout << "ur5_kendama_controller::configureHook" << std::endl;

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

        ip->MaxVelocityVector->VecData[i] = 1.0;
        ip->MaxAccelerationVector->VecData[i] = 1.0;
        ip->MaxJerkVector->VecData[i] = 1.0;

        ip->SelectionVector->VecData[i] = true;

        ip->TargetPositionVector->VecData[i] = 0.0;
        ip->TargetVelocityVector->VecData[i] = 0.0;
    }


    ball_launch_controller launchController(&ip, &fk_pos, &ik_pos, &ik_vel);
    ball_track_controller trackController(&ip, &fk_pos, &ik_pos, &ik_vel);

}


bool ur5_kendama_controller::startHook() 
{
    std::cout << "ur5_kendama_controller::startHook" << std::endl;
}


void ur5_kendama_controller::updateHook() 
{
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

    // If we have a commanded launch of the ball
    if( launchCommanded )
    {
        // Wait to achieve mid-waypoint
        KDL::JntArray diff(6);
        KDL::Subtract(joint_state, q_mid_desired, diff);

        float sum_of_squares = diff.data[0]*diff.data[0] + diff.data[1]*diff.data[1] + 
                    diff.data[2]*diff.data[2] + diff.data[3]*diff.data[3] + 
                    diff.data[4]*diff.data[4] + diff.data[5]*diff.data[5];

        float norm_diff = pow(sum_of_squares, 0.5);

        // If mid-waypoint achieved, proceed to end-waypoint to finish launch
        if( norm_diff < 0.1 || prevNorm < norm_diff )
        {
            launchBallLastSegment();
            launchCommanded = false;
            prevNorm = 1000;
        } 
        
        else 
        {
            prevNorm = norm_diff;
        }
    }


    // Get current position from FK
    //KDL::JntArray q_cur = joint_state;
    //KDL::Frame p_cur;
    //fk_pos->JntToCart(q_cur, p_cur, -1);
    // 0.817194, 0.303569
    //std::cout << p_cur.p.x() << ", " << p_cur.p.y() << ", " << p_cur.p.z() << std::endl; 
}


void ur5_kendama_controller::stopHook() 
{
    std::cout << "ur5_kendama_controller::stopHoop" << std::endl;
}


void ur5_kendama_controller::cleanupHook() 
{
    std::cout << "ur5_kendama_controller::cleanupHook" << std::endl;
}


KDL::JntArray ur5_kendama_controller::getJointPos()
{
    // return the current joint position
    KDL::JntArray js(6);
    for ( int i = 0; i < 6; i++ )
    {
        js.data[i] = ip->CurrentPositionVector->VecData[i];
    }

    return js;
}


void ur5_kendama_controller::setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot )
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
        ip->TargetPositionVector->VecData[i] = q.data[i];
        ip->TargetVelocityVector->VecData[i] = q_dot.data[i];
    }
}

void ur5_kendama_controller::commandTPose()
{
    KDL::JntArray t_pose_coords(6);
    t_pose_coords.data[0] = 0.96;
    t_pose_coords.data[1] = -1.04591;
    t_pose_coords.data[2] = 1.11256 ;
    t_pose_coords.data[3] = -0.0667037;
    t_pose_coords.data[4] = 1.5708;
    t_pose_coords.data[5] = 0.0;
    
    KDL::JntArray t_pose_velo(6);
    t_pose_velo.data[0] = 0.0;
    t_pose_velo.data[1] = 0.0;
    t_pose_velo.data[2] = 0.0;
    t_pose_velo.data[3] = 0.0;
    t_pose_velo.data[4] = 0.0;
    t_pose_velo.data[5] = 0.0;

    for ( int i = 0; i < 6; i++ )
    {
        ip->MaxVelocityVector->VecData[i] = 1.0;
        ip->MaxAccelerationVector->VecData[i] = 1.0;
        ip->MaxJerkVector->VecData[i] = 1.0;
    }

    setJointPos(t_pose_coords, t_pose_velo);
}

void ur5_kendama_controller::commandZeroPose()
{
    KDL::JntArray t_pose_coords(6);
    t_pose_coords.data[0] = 0.0;
    t_pose_coords.data[1] = 0.0;
    t_pose_coords.data[2] = 0.0;
    t_pose_coords.data[3] = 0.0;
    t_pose_coords.data[4] = 0.0;
    t_pose_coords.data[5] = 0.0;
    
    KDL::JntArray t_pose_velo(6);
    t_pose_velo.data[0] = 0.0;
    t_pose_velo.data[1] = 0.0;
    t_pose_velo.data[2] = 0.0;
    t_pose_velo.data[3] = 0.0;
    t_pose_velo.data[4] = 0.0;
    t_pose_velo.data[5] = 0.0;

    for ( int i = 0; i < 6; i++ )
    {
        ip->MaxVelocityVector->VecData[i] = 1.0;
        ip->MaxAccelerationVector->VecData[i] = 1.0;
        ip->MaxJerkVector->VecData[i] = 1.0;
    }

    setJointPos(t_pose_coords, t_pose_velo);
}

void ur5_kendama_controller::jointStateCallback(const sensor_msgs::JointState& js )
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

void ur5_kendama_controller::launchBallFirstSegment()
{
    // Get current position from FK
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    fk_pos->JntToCart(q_cur, p_cur, -1);

    // Compute mid-waypoint position
    KDL::Frame p_mid_desired = p_cur;
    KDL::Vector mid(0.0, 0.0, 0.05);
    p_mid_desired.p += mid;

    // Get mid-waypoint joint state from IK
    KDL::JntArray _q_mid_desired(6);
    ik_pos->CartToJnt(q_cur, p_mid_desired, _q_mid_desired);

    // Set mid-waypoint twist velo
    KDL::Vector rot(0.0, 0.0, 0.0);
    KDL::Vector vel(0.0, 0.0, 3.5);
    KDL::Twist v_desired;
    v_desired.rot = rot;
    v_desired.vel = vel;

    // Get mid-waypoint joint velo from IK_vel
    KDL::JntArray q_dot_desired(6);
    ik_vel->CartToJnt(_q_mid_desired, v_desired, q_dot_desired);

    std::cout << "Sending midpoint command..." << std::endl;

    std::cout << "Current joints: ";
    
    for ( int i = 0; i < 6; i++ )
    {
        std::cout << q_cur.data[i] << ", ";
    }

    std::cout << std::endl;
    
    // Set velo/accel limits
    for ( int i = 0; i < 6; i++ )
    {
        ip->MaxVelocityVector->VecData[i] = 15;
        ip->MaxAccelerationVector->VecData[i] = 150.0;
        ip->MaxJerkVector->VecData[i] = 400.0;
        std::cout << _q_mid_desired.data[i] << ", ";
    }

    std::cout << std::endl;

    // Send command for mid-waypoint
    setJointPos(_q_mid_desired, q_dot_desired);
    launchCommanded = true;
    q_mid_desired = _q_mid_desired;
}

void ur5_kendama_controller::launchBallLastSegment()
{
    // Get current position from FK
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    fk_pos->JntToCart(q_cur, p_cur, -1);

    // Compute end-waypoint position
    KDL::Frame p_end_desired = p_cur;
    KDL::Vector end(0.0, 0.0, 0.05);
    p_end_desired.p += end;

    // Get end-waypoint joint state from IK
    q_cur = joint_state;
    KDL::JntArray q_end_desired(6);
    ik_pos->CartToJnt(q_cur, p_end_desired, q_end_desired);

    // Set end-waypoint joint velo
    KDL::JntArray zero_vel(6);
    zero_vel.data[0] = 0.0;
    zero_vel.data[1] = 0.0;
    zero_vel.data[2] = 0.0;
    zero_vel.data[3] = 0.0;
    zero_vel.data[4] = 0.0;
    zero_vel.data[5] = 0.0;

    std::cout << "Sending endpoint command..." << std::endl;

    // Set velo/accel limits
    for ( int i = 0; i < 6; i++ )
    {
        ip->MaxVelocityVector->VecData[i] = 15;
        ip->MaxAccelerationVector->VecData[i] = 150.0;
        ip->MaxJerkVector->VecData[i] = 400.0;
        std::cout << q_end_desired.data[i] << ", ";
    }

    std::cout << std::endl << std::endl;

    // Send command for mid-waypoint
    setJointPos(q_end_desired, zero_vel);
}


ORO_CREATE_COMPONENT(ur5_kendama_controller) // register the RTT component
