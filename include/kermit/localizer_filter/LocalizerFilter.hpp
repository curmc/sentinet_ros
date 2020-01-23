/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : LocalizerFilter
 * @brief       : A brief description of the file
 *
 * A detailed description of the file
 *
 * @created     : Tuesday Jan 07, 2020 15:54:16 MST
 * @license     : MIT
 */

#ifndef LOCALIZERFILTER_HPP

#define LOCALIZERFILTER_HPP

#include "kermit/common.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "kermit/mars_imu.h"
#include "kermit/path_state.h"

class LocalizerFilter
{
  public:
    LocalizerFilter ();
    virtual ~LocalizerFilter ();
  private:
    void imu_callback (const kermit::mars_imu& msg);
    void camera_callback (const geometry_msgs::PoseStamped& camera);

    // Filter Stuff
    struct {
      kermit::path_state path_state;
      kermit::path_state path_state_prev;
    } state;

    ros::NodeHandle nh;
    
    ros::Subscriber imu;
    ros::Subscriber camera;
    ros::Publisher path_state;
};

#endif /* end of include guard LOCALIZERFILTER_HPP */

