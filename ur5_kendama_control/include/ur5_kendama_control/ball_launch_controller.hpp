#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <ReflexxesTypeII/ReflexxesAPI.h>
#include <ReflexxesTypeII/RMLPositionInputParameters.h>


// This is the component that actuall connects to the hardware
class ball_launch_controller {

    private :

        ros::NodeHandle nh;
        ros::Subscriber sub_js;

        RMLPositionInputParameters** ip_ref;

        KDL::ChainFkSolverPos** fk_pos_ref;
        KDL::ChainIkSolverPos** ik_pos_ref;
        KDL::ChainIkSolverVel** ik_vel_ref;

        bool launchFinished;
        bool launchFinishedFirstSegment;

        KDL::JntArray q_commanded;
        KDL::JntArray joint_state;

        float prevNorm;


    public : 

        ball_launch_controller( ros::NodeHandle& nh );
        ~ball_launch_controller() {}

        void initializeParameters( RMLPositionInputParameters** ip, KDL::ChainFkSolverPos** fk_pos, 
                KDL::ChainIkSolverPos** ik_pos, KDL::ChainIkSolverVel** ik_vel );

        void updateHook();

        void setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot );
        void jointStateCallback(const sensor_msgs::JointState& js );

        void launchBallFirstSegment();
        void launchBallLastSegment();

        bool finishedLaunch();

};