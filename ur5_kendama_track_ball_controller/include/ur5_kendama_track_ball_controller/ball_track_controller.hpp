#include <ros/ros.h>

#include <ur5_kendama_track_ball_controller/ball_position_subscriber.hpp> 

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

#include <ReflexxesTypeII/ReflexxesAPI.h>
#include <ReflexxesTypeII/RMLPositionFlags.h>
#include <ReflexxesTypeII/RMLPositionInputParameters.h>
#include <ReflexxesTypeII/RMLPositionOutputParameters.h>


class ball_track_controller : public RTT::TaskContext {

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

        KDL::JntArray joint_state;

        ball_pos_subscriber ball_pos_sub;

        float robotHeight;
        KDL::Rotation robotOrientation;

        bool firstJntState;


    public : 

        ball_track_controller( const std::string& name );
        ~ball_track_controller() {}

        virtual bool configureHook();
        virtual bool startHook();

        virtual void updateHook();

        virtual void stopHook();
        virtual void cleanupHook();

        KDL::JntArray getJointPos();
        void setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot );

        void jointStateCallback(const sensor_msgs::JointState& js );

        void initializeHeight(float z);

        void trackBall(float ball_x, float ball_y);

};