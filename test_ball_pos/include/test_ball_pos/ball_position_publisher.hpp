#include <ros/ros.h>

// Class definition for publisher class
class ball_pos_publisher {
    
    private:

        ros::NodeHandle nh;
        ros::Publisher pub;

    public:

        ball_pos_publisher( ros::NodeHandle& nh );
        ~ball_pos_publisher();

        void publish();

};