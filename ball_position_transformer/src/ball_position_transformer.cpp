#include <ros/ros.h>
#include <ur5_kendama_msgs/ball_position.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <kdl/frames.hpp>

ur5_kendama_msgs::ball_position world_ball_pos_msg;
geometry_msgs::TransformStamped camera_to_world;
ros::Publisher pub;

void ballPosCallback(const ur5_kendama_msgs::ball_position& camera_ball_pos) {

  //Apply transformation to ball position in camera frame
  //TODO
  geometry_msgs::PointStamped camera_ball_point;
  camera_ball_point.point.x = camera_ball_pos.x;
  camera_ball_point.point.y = camera_ball_pos.y;
  camera_ball_point.point.z = camera_ball_pos.z;

  geometry_msgs::PointStamped world_ball_point;
  tf2::doTransform(camera_ball_point, world_ball_point, camera_to_world);

  world_ball_pos_msg.x = world_ball_point.point.x;
  world_ball_pos_msg.y = world_ball_point.point.y;
  world_ball_pos_msg.z = world_ball_point.point.z;

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
