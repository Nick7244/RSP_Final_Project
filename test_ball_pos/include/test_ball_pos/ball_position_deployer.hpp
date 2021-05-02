#include <ros/ros.h>

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/jntarray.hpp>
#include <std_msgs/Float64MultiArray.h>

#include <ur5_kendama_msgs/ball_position.h>

// Class definition for publisher class
class ball_pos_deployer : public RTT::TaskContext {
    
    private:

        ros::NodeHandle nh;

        RTT::OutputPort<std_msgs::Float64MultiArray> port_ball_pos;

        std_msgs::Float64MultiArray ball_pos;

    public:

        ball_pos_deployer( const std::string& name );
        ~ball_pos_deployer();

        virtual bool configureHook();
        virtual bool startHook();

        virtual void updateHook();

        virtual void stopHook();
        virtual void cleanupHook();

        void publish();

        KDL::JntArray getBallPos();
        void setBallPos( KDL::JntArray ball_pos_ary );

};