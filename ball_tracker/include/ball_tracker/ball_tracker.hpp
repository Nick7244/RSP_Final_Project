//Ball tracker header file

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <visp/vpConfig.h>
#include <visp/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp_bridge/image.h>


class ball_tracker {

private:
  ros::NodeHandle nh;
  
  ros::Subscriber sub_color;
  ros::Subscriber sub_depth;

  ros::Publisher pub_pos;

  sensor_msgs::Image color_img;
  sensor_msgs::Image depth_img;

  bool color_received;
  bool depth_received;

  bool visp_init_done;
  vpDot2 blob;
  vpImagePoint germ;

public:
  ball_tracker(ros::NodeHandle& nh);
  void color_callback(const sensor_msgs::Image& incoming_color_img);
  void depth_callback(const sensor_msgs::Image& incoming_depth_img);

  void visp_tracking();

};
