#include <ros/ros.h>
#include <ur5_kendama_msgs/ball_position.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <gemetry_msgs/Pose.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <kdl/frames.hpp>

ur5_kendama_msgs::ball_position world_cube_pos_msg;
geometry_msgs::TransformStamped camera_to_world;
ros::Publisher pub;

void cubePosCallback(const geometry_msgs::PoseStamped& camera_cube_pos) {

  //Apply transformation to ball position in camera frame
  geometry_msgs::PointStamped camera_cube_point;
  camera_cube_point.point = camera_cube_pos.pose.position;

  geometry_msgs::PointStamped world_cube_point;
  tf2::doTransform(camera_cube_point, world_cube_point, camera_to_world);

  world_cube_pos_msg.x.data = world_cube_point.point.x;
  world_cube_pos_msg.y.data = world_cube_point.point.y;
  world_cube_pos_msg.z.data = world_cube_point.point.z;

  //Publish ball position in world frame
  pub.publish(world_cube_pos_msg);
  
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "cube_pos_transformer_node");

  ros::NodeHandle nh;

  // Get transformation from camera frame to world frame
  camera_to_world = *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/calibration_result"));
  
  // Subscribe to topic publishing ball's position in camera frame
  ros::Subscriber sub = nh.subscribe("/aruco/pose", 10, &cubePosCallback);

  pub = nh.advertise<ur5_kendama_msgs::ball_position>("/ball_position", 10);

  ros::spin();
  return 0;

}
