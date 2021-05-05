#include <visp/vpConfig.h>
#include <visp/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp_bridge/image.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>


int main(int argc, char **argv) {

  ros::init(argc, argv, "get_images");
  
  sensor_msgs::Image color_img;
  sensor_msgs::Image depth_img;
 
  color_img = *(ros::topic::waitForMessage<sensor_msgs::Image>("camera/color/image_raw"));
  depth_img = *(ros::topic::waitForMessage<sensor_msgs::Image>("camera/depth/image_raw"));
 
  vpImage<vpRGBa> I_rgb = visp_bridge::toVispImageRGBa(color_img);
  vpImage<unsigned char> I;
  vpImageConvert::convert(I_rgb, I);

  vpDisplayX d(I, 0, 0, "Camera view");
  
  vpDot2 blob;
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);

  vpImagePoint germ;
  bool init_done = false;
  std::cout << "Click!!!" << std::endl;
  while (1) {
    try {
      color_img = *(ros::topic::waitForMessage<sensor_msgs::Image>("camera/color/image_raw"));
      depth_img = *(ros::topic::waitForMessage<sensor_msgs::Image>("camera/depth/image_raw"));
       
      I_rgb = visp_bridge::toVispImageRGBa(color_img);
      vpImageConvert::convert(I_rgb, I);

      vpDisplay::display(I);

      if (!init_done) {
        vpDisplay::displayText(I, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
        if (vpDisplay::getClick(I, germ, false)) {
          blob.initTracking(I, germ);
          init_done = true;
        }
      } else {
        blob.track(I);
	vpImagePoint centroid = blob.getCog();
	double x = centroid.get_i();
	double y = centroid.get_j();
	uint16_t z = depth_img.data[int(x)*depth_img.step + int(y)];
	std::cout << "Coordinates: (" << x << ", " << y << ", " << z << ")";
      }
      vpDisplay::flush(I);
    } catch (...) {
      init_done = false;
    }
  }

}
