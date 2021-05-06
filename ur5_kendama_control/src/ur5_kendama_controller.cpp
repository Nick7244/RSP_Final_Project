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
    prevNorm(1000),
    launchController(nh),
    trackController(nh),
    launchCommanded(false),
    trackCommanded(false),
    initialLaunch(true),
    TPoseCommanded(false),
    firstErrorMsg(true),
    errorState(false)
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
    addOperation("LaunchBall", &ur5_kendama_controller::launchBall, this, RTT::OwnThread);
    addOperation("TrackBall", &ur5_kendama_controller::trackBall, this, RTT::OwnThread);
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

        ip->MaxVelocityVector->VecData[i] = 0.5;
        ip->MaxAccelerationVector->VecData[i] = 0.75;
        ip->MaxJerkVector->VecData[i] = 1.0;

        ip->SelectionVector->VecData[i] = true;

        ip->TargetPositionVector->VecData[i] = 0.0;
        ip->TargetVelocityVector->VecData[i] = 0.0;
    }


    launchController.initializeParameters(&ip, &fk_pos, &ik_pos, &ik_vel);
    trackController.initializeParameters(&ip, &fk_pos, &ik_pos, &ik_vel);

}


bool ur5_kendama_controller::startHook() 
{
    std::cout << "ur5_kendama_controller::startHook" << std::endl;
}


void ur5_kendama_controller::updateHook() 
{
    // If we are launching the ball
    if ( launchCommanded )
    {
        // If we are not yet done launching, call the launch controller update hook
        if ( !launchController.finishedLaunch() )
        {
            launchController.updateHook();
        }

        // Else, we are done launching and will begin tracking the ball
        else
        {
            std::cout << "Ball successfully launched!" << std::endl;
            launchCommanded = false;

            bool launch_only;
            nh.param("/launch_only", launch_only, bool());

            if ( !launch_only )
            {
                trackBall();
            }
        }
        
    }

    // If we are tracking the ball, call the track controller update hook
    if ( trackCommanded )
    {
        trackController.updateHook();
    }


    // compute an iteration of the trajectory
    int result = rml->RMLPosition(*ip, op, flags);

    // setup for next iteration
    *ip->CurrentPositionVector = *op->NewPositionVector;
    *ip->CurrentVelocityVector = *op->NewVelocityVector;
    *ip->CurrentAccelerationVector = *op->NewAccelerationVector;

    // get the joint values and write them to the port
    std_msgs::Float64MultiArray js;
    js.data.resize(6);

    KDL::JntArray q_next(6);
        
    for ( int i = 0; i < 6; i++ )
    {
        op->GetNewPositionVectorElement(&js.data[i], i);
        q_next.data[i] = js.data[i];
    }

    // Get current cartesian position of end effector
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    fk_pos->JntToCart(q_cur, p_cur, -1);
    float cupHeight =  p_cur.p.z();

    KDL::Frame p_next;
    fk_pos->JntToCart(q_next, p_next, -1);
    float nextHeight = p_next.p.z();

    if ( (!(cupHeight <= 0.15 && nextHeight < cupHeight) || initialLaunch) && !errorState )
    {
        port_msr_jnt_state.write(js);

        if ( initialLaunch && TPoseCommanded )
        {
            initialLaunch = false;
        }
    }

    else
    {
        if ( firstErrorMsg )
        {
            ROS_INFO("Attempting to move robot too low, robot will not move further. Please reinitialize to T-pose.");
            firstErrorMsg = false;
            errorState = true;
        }
    }

    
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
        ip->MaxVelocityVector->VecData[i] = 0.5;
        ip->MaxAccelerationVector->VecData[i] = 0.75;
        ip->MaxJerkVector->VecData[i] = 1.0;

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

    launchCommanded = false;
    trackCommanded = false;
    firstErrorMsg = true;
    errorState = false;
    TPoseCommanded = true;

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

    launchCommanded = false;
    trackCommanded = false;

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

void ur5_kendama_controller::launchBall()
{
    launchCommanded = true;
    trackCommanded = false;

    trackController.resetTrack();
    launchController.launchBallFirstSegment();
}

void ur5_kendama_controller::trackBall()
{
    bool launch_only;
    nh.param("/launch_only", launch_only, bool());

    if ( !launch_only )
    {
        launchCommanded = false;
        trackCommanded = true;

        KDL::JntArray q_cur = joint_state;
        KDL::Frame p_cur;
        fk_pos->JntToCart(q_cur, p_cur, -1);
    
        trackController.initializeHeight(p_cur.p.z(), p_cur.M);
    }

    else
    {
        ROS_INFO("Attempting to run the ball tracking controller in 'Launch Only' mode.");
        ROS_INFO("If you wish to run the ball tracking controller, please disable 'Launch Only' mode.");
    }
    
}


ORO_CREATE_COMPONENT(ur5_kendama_controller) // register the RTT component
