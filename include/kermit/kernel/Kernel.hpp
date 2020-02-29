/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : Kernel
 * @brief       : A brief description of the file
 *
 * A detailed description of the file
 *
 * @created     : Monday Jan 06, 2020 18:57:53 MST
 * @license     : MIT
 */

#ifndef KERNEL_HPP

#define KERNEL_HPP

// Kermit Includes
#include "kermit/common.h"
#include "kermit/kernel/teensy_serial.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// ROS Includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// #include "kermit/mars_imu.h"

// C++ Includes
#include <chrono>
#include <string>
#include <thread>
#include <memory>

using namespace std::chrono;

class Kernel {
      public:
        /**
         * @brief Default constructor
         *
         * @param debug If true, Logs info everytime a cmd_Vel is recieved
         * @param simulation_ If true, creates a simulation of kermit
         */
        Kernel();
        ~Kernel();

      public:

        // Serial only activates if explicitly called, otherwise logs to the screen
        bool initialize_teensy(const std::string &port);

        // CALLBACKS
      private:
        // Recieve a command from controller
        void cmd_vel_callback(const geometry_msgs::Twist &msg);

        // Utility printer
        void debug_printf(const geometry_msgs::Twist &msg);

      private:

        // Handle Teensy Communications
        bool teensy_callback(float lin, float ang);

        // Main node handle
        ros::NodeHandle n;

        // Subscriber to cmd_vel
        ros::Subscriber sub;

        // publisher to localizer
        ros::Publisher pub;

        to_teensy_msg to_msg;
        from_teensy_msg from_msg;;
        serial_dev_attr dev;
};

#endif /* end of include guard KERNEL_HPP */
