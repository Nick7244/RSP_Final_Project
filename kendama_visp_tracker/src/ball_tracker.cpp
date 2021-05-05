//Ball tracker

#include <kendama_visp_tracker/ball_tracker.hpp>
#include <visp/vpConfig.h>
#include <visp/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp_bridge/image.h>

#include <visp/vpImagePoint.h>
#include <visp/vpImageIo.h>

#include <unistd.h>

ball_tracker::ball_tracker(ros::NodeHandle& nh) 
  : nh(nh) ,
  color_received(false),
  depth_received(false),
  visp_init_done(false)
{
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);

  sub_color = nh.subscribe("/camera/color/image_raw", 10, &ball_tracker::color_callback, this);
  sub_depth = nh.subscribe("/camera/depth/image_raw", 10, &ball_tracker::depth_callback, this);

  

    vpImage<unsigned char> I; // Grey level image
  // Read an image in PGM P5 format
  vpImageIo::read(I, "/home/nick-maritato/ball.png");
#if defined(VISP_HAVE_X11)
  vpDisplayX d;
  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);
#endif
  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);
  // Set the display window title
  vpDisplay::setTitle(I, "My X11 display");
  // Set the display background with image I content
  vpDisplay::display(I);
  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);
  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(50);
  topLeftCorner.set_j(10);
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::green, true);
  // Flush the foreground and background display
  vpDisplay::flush(I);
  // Get non blocking keyboard events
  std::cout << "Check keyboard events..." << std::endl;
  char key[10];
  bool ret;
  for (int i=0; i< 200; i++) {
    bool ret = vpDisplay::getKeyboardEvent(I, key, false);
    if (ret)
      std::cout << "keyboard event: key: " << "\"" << key << "\"" << std::endl;
    vpTime::wait(40);
  }
  // Get a blocking keyboard event
  std::cout << "Wait for a keyboard event..." << std::endl;
  ret = vpDisplay::getKeyboardEvent(I, key, true);
  std::cout << "keyboard event: " << ret << std::endl;
  if (ret)
    std::cout << "key: " << "\"" << key << "\"" << std::endl;
  // Wait for a click in the display window
  std::cout << "Wait for a button click..." << std::endl;
  vpDisplay::getClick(I);
  

}

void ball_tracker::color_callback(const sensor_msgs::Image& incoming_color_img) 
{

  /*if(this->color_received == false) {
    ROS_INFO("Color image received. Waiting for depth image.");
    this->color_img = incoming_color_img;
    this->color_received = true;
  } else {
    ROS_INFO("Color image already received.");
  }*/

}


void ball_tracker::depth_callback(const sensor_msgs::Image& incoming_depth_img) {

  /*if((this->color_received == true) && (this->depth_received == false)) {
    this->depth_img = incoming_depth_img;
    this->depth_received = true;
    //Visp tracking
    ROS_INFO("Depth image receieved. Time for Visp stuff.");
    visp_tracking();
  }*/
  
}

void ball_tracker::visp_tracking() {

  /*ROS_INFO("In function ball_tracker::visp_tracking.");
  vpImage<vpRGBa> I_rgb = visp_bridge::toVispImageRGBa(this->color_img);
  vpImage<unsigned char> I;
  vpImageConvert::convert(I_rgb, I);
  vpDisplayX d(I, 0, 0, "Camera view");

  try {
    vpDisplay::display(I);
    
    if(!this->visp_init_done) {
      std::cout << "Click blob to initialize tracker" << std::endl;
      vpDisplay::displayText(I, vpImagePoint(10,10), "Click blob to initialize tracker", vpColor::red);
      sleep(5);
      if(vpDisplay::getClick(I, this->germ, false)) {
	this->blob.initTracking(I, this->germ);
	this->visp_init_done = true;
      }
    
    } else {

      this->blob.track(I);
      vpImagePoint centroid = this->blob.getCog();
      std::cout << "Centroid coords: (" << centroid.get_i() << ", " << centroid.get_j() << ")" << std::endl;
    
    }
    vpDisplay::flush(I);
  } catch(...) {
    this->visp_init_done = false;
  }

  this->color_received = false;
  this->depth_received = false;
  
  std::cout << "End of ball_tracker::visp_function" << std::endl;*/
  
}
