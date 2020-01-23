/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : kernel
 * @created     : Tuesday Jan 07, 2020 13:43:04 MST
 */

#include "kermit/kernel/Kernel.hpp"


int main(int argc, char** argv) {


  ros::init(argc, argv, "Kernel");

  Kernel kernel(true, false);
  kernel.initialize_teensy("/dev/ttyACM0");

  ros::spin();

  return 0;

}


