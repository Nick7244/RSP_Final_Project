#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

#include <ReflexxesTypeII/ReflexxesAPI.h>
#include <ReflexxesTypeII/RMLPositionFlags.h>
#include <ReflexxesTypeII/RMLPositionInputParameters.h>
#include <ReflexxesTypeII/RMLPositionOutputParameters.h>

#include <ur5_kendama_launch_controller/ball_launch_controller.hpp>
#include <ur5_kendama_track_ball_controller/ball_track_controller.hpp>


// This is the component that actuall connects to the hardware
class ur5_kendama_controller : public RTT::TaskContext {

    private :

        ros::NodeHandle nh;
        ros::Subscriber sub_js;

        RTT::OutputPort<std_msgs::Float64MultiArray> port_msr_jnt_state;

        ReflexxesAPI *rml;
        RMLPositionInputParameters* ip;
        RMLPositionOutputParameters* op;
        RMLPositionFlags flags;

        KDL::ChainFkSolverPos* fk_pos;
        KDL::ChainIkSolverPos* ik_pos;
        KDL::ChainIkSolverVel* ik_vel;

        KDL::Tree tree;
        KDL::Chain chain;

        bool launchCommanded;
        KDL::JntArray q_mid_desired;

        KDL::JntArray joint_state;

        float prevNorm;


    public : 

        ur5_kendama_controller( const std::string& name );
        ~ur5_kendama_controller() {}

        virtual bool configureHook();
        virtual bool startHook();

        virtual void updateHook();

        virtual void stopHook();
        virtual void cleanupHook();

        KDL::JntArray getJointPos();
        void setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot );

        void commandTPose();
        void commandZeroPose();

        void jointStateCallback(const sensor_msgs::JointState& js );

        void launchBallFirstSegment();
        void launchBallLastSegment();

};