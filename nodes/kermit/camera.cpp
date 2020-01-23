/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : camera
 * @created     : Tuesday Jan 07, 2020 15:03:51 MST
 */

#include "kermit/camera/AprilDetector.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "Camera");
  
  AprilDetector detector("tag36h11");

  detector.sync_start();

  ros::Rate loop_rate(100);

  while(ros::ok()) {
    detector.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
