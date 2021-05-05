#include <visp/vpConfig.h>
#include <visp/vpV4l2Grabber.h>
#include <visp/vpDot2.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <unistd.h>

int main(int argc, char **argv) {

  int device = 3;
  if (argc == 2) {
    device = std::atoi(argv[1]);
  }
  vpImage<unsigned char> I; // Create a gray level image container

  std::stringstream ss;
  ss << "/dev/video" << device;

  std::cout << "Connect to: " << ss.str() << std::endl;

  std::cout << "visp have v4l2" << std::endl;
  vpV4l2Grabber g;
  g.setDevice(ss.str());
  g.open(I);


  vpDisplayX d(I, 0, 0, "Camera view");
  std::cout << "x11" << std::endl;

  sleep(3);
  
  vpDot2 blob;
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);

  vpImagePoint germ;
  bool init_done = false;
  std::cout << "Click!!!" << std::endl;
  while (1) {
    try {
      g.acquire(I);
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
	std::cout << "Centroid coords: (" << centroid.get_i() << ", " << centroid.get_j() << ")" << std::endl;
      }
      vpDisplay::flush(I);
    } catch (...) {
      init_done = false;
    }
  }

}
