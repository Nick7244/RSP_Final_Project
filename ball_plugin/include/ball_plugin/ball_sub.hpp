#include <ros/ros.h>
#include <ur5_kendama_msgs/ball_position.h>
#include <vector>

// Class definition for subscriber class
class ball_pos_subscriber {

    private:

        ros::NodeHandle nh;
        ros::Subscriber sub;
        std::vector<float> ball_pos;

    public: 

        ball_pos_subscriber( ros::NodeHandle& nh );
        ~ball_pos_subscriber();

        void callback( const ur5_kendama_msgs::ball_position& msg );
};