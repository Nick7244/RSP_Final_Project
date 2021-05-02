#include <ros/ros.h>
#include <ur5_kendama_msgs/ball_position.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

// Class definition for publisher class
class ball_pos_publisher {
    
    private:

        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber deployerSub;

        ur5_kendama_msgs::ball_position myMessage;

    public:

        ball_pos_publisher( ros::NodeHandle& nh );
        ~ball_pos_publisher();

        void publish();
        void callback( const std_msgs::Float64MultiArray& ball_pos_ary );

};