#include <ros/ros.h>
#include <ur5_kendama_msgs/ball_position.h>
#include <geometry_msgs/TransformStamped.h>

ur5_kendama_msgs::ball_position world_ball_pos_msg;
geometry_msgs::TransformStamped camera_to_world;
ros::Publisher pub;

void ballPosCallback(const ur5_kendama_msgs::ball_position& camera_ball_pos) {

  //Apply transformation to ball position in camera frame
  //TODO

  //Publish ball position in world frame
  pub.publish(world_ball_pos_msg);
  
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "ball_pos_transformer_node");

  ros::NodeHandle nh;

  // Get transformation from camera frame to world frame
  camera_to_world = *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/calibration_result"));
  
  // Subscribe to topic publishing ball's position in camera frame
  ros::Subscriber sub = nh.subscribe("/ball_pos_camera", 10, &ballPosCallback);

  pub = nh.advertise<ur5_kendama_msgs::ball_position>("ball_pos_world", 10);

  ros::spin();
  return 0;

}
