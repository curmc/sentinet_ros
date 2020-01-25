/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : PosLocalizer
 * @brief       : A brief description of the file
 *
 * A detailed description of the file
 *
 * @created     : Wednesday Jan 01, 2020 14:00:17 MST
 * @license     : MIT
 */

#ifndef POSLOCALIZER_HPP

#define POSLOCALIZER_HPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PosLocalizer {
      public:
        PosLocalizer();
        ~PosLocalizer();

      protected:
        // Prints x y and theta
        void print_state();

      protected:
        bool verbose;

      private:
        // The publisher callback
        std::string get_pos(void);

      private:
        struct {
                geometry_msgs::Quaternion quat_msg;
                geometry_msgs::Point point;
        } pos;
};

#endif /* end of include guard POSLOCALIZER_HPP */
