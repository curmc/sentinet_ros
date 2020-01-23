/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : smath
 * @created     : Monday Jan 06, 2020 11:51:27 MST
 */

#ifndef SMATH_H

#define SMATH_H

#include <cmath>
#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
}

using namespace cv;

typedef struct {
  double roll;
  double pitch;
  double yaw;
} orientation;

// OPENCV Functions
bool cvIsRotationMatrix(const Mat &R);
orientation cvRotationMatrixToEulerAngles(const Mat &R);
Mat cvEulerAnglesToRotationMatrix(const orientation &theta);

// April Tags functions
// bool aprilIsRotationMatrix(const matd_t* R);
orientation aprilRotationMatrixToEulerAngles(const matd_t* R);
// matd_t aprilEulerAnglesToRotationMatrix(const orientation* o);



#endif /* end of include guard SMATH_H */

