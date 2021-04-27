#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>

#include <kdl/jntarray.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

#include <ReflexxesTypeII/ReflexxesAPI.h>
#include <ReflexxesTypeII/RMLPositionFlags.h>
#include <ReflexxesTypeII/RMLPositionInputParameters.h>
#include <ReflexxesTypeII/RMLPositionOutputParameters.h>

enum ur5_controller_state 
{
    idle,
    launch,
    follow,
    craddle
};

// This is the component that actuall connects to the hardware
class ball_launch_controller : public RTT::TaskContext {

    private :

        RTT::OutputPort<std_msgs::Float64MultiArray> port_msr_jnt_state;

        ReflexxesAPI *rml;
        RMLPositionInputParameters* ip;
        RMLPositionOutputParameters* op;
        RMLPositionFlags flags;

        ur5_controller_state controller_state;


    public : 

        ball_launch_controller( const std::string& name );
        ~ball_launch_controller() {}

        virtual bool configureHook();
        virtual bool startHook();

        virtual void updateHook();

        virtual void stopHook();
        virtual void cleanupHook();

        KDL::JntArray getJointPos();
        void setJointPos( const KDL::JntArray& q , const KDL::JntArray& q_dot );

        void commandTPose();
        void commandZeroPose();

};