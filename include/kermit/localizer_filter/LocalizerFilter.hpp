/**
 * @author                        : theo (theo.j.lincke@gmail.com)
 * @file                                : LocalizerFilter
 * @brief                         : A brief description of the file
 *
 * A detailed description of the file
 *
 * @created                 : Tuesday Jan 07, 2020 15:54:16 MST
 * @license                 : MIT
 */

#ifndef LOCALIZERFILTER_HPP

#define LOCALIZERFILTER_HPP

#include "kermit/common.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class LocalizerFilter {
      public:
        LocalizerFilter();
        virtual ~LocalizerFilter();

      private:
        // Register a callback for incomming data
        template <class SDT>
        void register_sensor_callback(std::function<void(SDT)> callback);

        // Get geometry message to publish onto the wire
        const geometry_msgs::Twist get_data();

      private:
        ros::NodeHandle nh;

        std::map < std::function<void(SDT)>
};

#endif /* end of include guard LOCALIZERFILTER_HPP */
