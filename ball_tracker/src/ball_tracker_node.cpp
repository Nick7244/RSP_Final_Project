// Ball tracker node

#include <ball_tracker/ball_tracker.hpp>

int main(int argc, char** argv) {

  ros::init(argc, argv, "ball_tracker");
  ros::NodeHandle nh;

  ball_tracker tracker_sub(nh);

  ros::spin();
  return 0;

}
