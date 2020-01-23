/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : wii_controller
 * @created     : Wednesday Jan 08, 2020 15:31:25 MST
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>

#include "kermit/common.h"
#include "kermit/controller/wii/kermit.h"

struct robot_s robot_main;

void robot_exit() {
  robot_clean_up(&robot_main);
}

int main(int argc, char** argv) {

  atexit(robot_exit);

  // Initialize ros
  ros::init(argc, argv, "WiiController");
  ros::NodeHandle n;

  // Publisher for cmd_vels
  ros::Publisher p = n.advertise<geometry_msgs::Twist>(topics::cmd_vel_topic, 1000);

  // Create kermit (from kermit.h)
  robot_main = kermit_robot();

  robot_unsetopt(&robot_main, DEBUG);
  robot_setopt(&robot_main, VERBOSE);
  robot_unsetopt(&robot_main, INSYNC);
  robot_setopt(&robot_main, VAR_SPEED);
  robot_unsetopt(&robot_main, ADVNCD);
  robot_unsetopt(&robot_main, DISCLINANG);
  robot_unsetopt(&robot_main, NONLIN);

  wiimote **wiimotes = scan_wii();
  struct controller_s controller = kermit_controller();
  
  ros::Rate loop_rate(100);
  geometry_msgs::Twist twist;
  
  float scale = sqrt(2);

  while(ros::ok()) {
    if(heart_beat(wiimotes, 1)) {

      event_loop(wiimotes, &robot_main, &controller);
      float v = robot_main.drive->linear_vel;
      float w = robot_main.drive->angular_vel;

      twist.linear.x = v / scale;
      twist.linear.y = v / scale;

      twist.angular.z = w;

      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.linear.z = 0.0;

      p.publish(twist);

    } else {
      ROS_WARN("No wiimote heartbeat detected");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
