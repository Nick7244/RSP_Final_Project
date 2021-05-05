#include <ros/ros.h>

#include <ur5_kendama_track_ball_controller/ball_position_subscriber.hpp> 

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <ReflexxesTypeII/ReflexxesAPI.h>
#include <ReflexxesTypeII/RMLPositionInputParameters.h>


class ball_track_controller {

    private :

        ros::NodeHandle nh;
        ros::Subscriber sub_js;

        RMLPositionInputParameters** ip_ref;

        KDL::ChainFkSolverPos** fk_pos_ref;
        KDL::ChainIkSolverPos** ik_pos_ref;
        KDL::ChainIkSolverVel** ik_vel_ref;

        KDL::JntArray joint_state;

        ball_pos_subscriber ball_pos_sub;

        float robotHeight;
        KDL::Rotation robotOrientation;

        bool firstJntState;
        bool firstTrack;
        bool heightInitialized;

        double timeLastTrack;


    public : 

        ball_track_controller( RMLPositionInputParameters** ip, KDL::ChainFkSolverPos** fk_pos, 
                KDL::ChainIkSolverPos** ik_pos, KDL::ChainIkSolverVel** ik_vel );

        ~ball_track_controller() {}

        virtual void updateHook();
        
        void jointStateCallback( const sensor_msgs::JointState& js );
        void initializeHeight( float z, KDL::Rotation orientation );
        void setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot );
        void trackBall( float ball_x, float ball_y, float ball_z, double ballVertVelo );

};