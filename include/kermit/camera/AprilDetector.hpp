/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : april_pos_estimator
 * @created     : Tuesday Dec 31, 2019 18:23:20 MST
 */

#ifndef APRIL_POS_ESTIMATOR_H

#define APRIL_POS_ESTIMATOR_H

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "kermit/common.h"


extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
}

using namespace cv;

#define DEFAULT_TAG "tag36h11"

class AprilDetector 
{
public:
  AprilDetector(const std::string type = DEFAULT_TAG);
  ~AprilDetector();

  // Starts the camera
  bool sync_start();
  // Loops camera logic
  bool loop();

private:
  // Used for default destructor
  void destroy_video();
  void destroy_tag();


  // April Tag specific init options
  bool initialize_tag(const std::string type);

  // CV Namespace
  struct
  {
    VideoCapture cap;
    Mat frame;
    Mat gray;
  } video;

  // April tag namespace
  struct
  {
    int type;
    apriltag_family_t* tf;
    apriltag_detector_t* td;
    zarray_t* detections;
    apriltag_detection_info_t info;
  } tag;

  // Message Namespace

  geometry_msgs::PoseStamped message; 
  ros::NodeHandle n;
  ros::Publisher pub;
};

#endif /* end of include guard APRIL_POS_ESTIMATOR_H */
