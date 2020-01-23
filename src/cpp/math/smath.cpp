/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : smath
 * @created     : Monday Jan 06, 2020 11:58:36 MST
 */

#include "kermit/math/smath.h"


static double R_at(const matd_t* R, size_t r, size_t c){
  return R->data[r * R->ncols + c];
}

orientation aprilRotationMatrixToEulerAngles(const matd_t* R) {
  float sy = sqrt(R_at(R, 0, 0) * R_at(R, 0, 0) + R_at(R, 1, 0) * R_at(R, 1, 0));
  bool singular = sy < 1e-6;
  float x, y, z;

  if(!singular) {
    x = atan2(R_at(R, 2, 1), R_at(R, 2, 2));
    y = atan2(-R_at(R, 2, 0), sy);
    z = atan2(R_at(R, 1, 0), R_at(R, 0, 0));
  } else {
    x = atan2(-R_at(R, 1, 2), R_at(R, 1, 1));
    y = atan2(-R_at(R, 2, 0), sy);
    z = 0;
  }

  return (orientation) {
          .roll = x,
          .pitch = y,
          .yaw = z};
}

bool cvIsRotationMatrix(const Mat &R) {
  Mat Rt;
  transpose(R, Rt);
  Mat I_ = Rt * R;
  Mat I = Mat::eye(3, 3, I_.type());
  return norm(I, I_) < 1e-6;
}

orientation cvRotationMatrixToEulerAngles(const Mat &R){
  assert(cvIsRotationMatrix(R));
  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6;
  float x, y, z;

  if(!singular)
  {
    x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
    y = atan2(-R.at<double>(2,0), sy);
    z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }
  else
  {
    x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
    y = atan2(-R.at<double>(2,0), sy);
    z = 0;
  }

  return (orientation) {
          .roll = x,
          .pitch = y,
          .yaw = z};
}

Mat cvEulerAnglesToRotationMatrix(const orientation &theta) {
  double sx = sin(theta.roll);
  double cx = cos(theta.roll);

  double sy = sin(theta.pitch);
  double cy = cos(theta.pitch);

  double sz = sin(theta.yaw);
  double cz = cos(theta.pitch);

  Mat R_x = (Mat_<double>(3, 3) << 
    1, 0, 0,
    0, cx, -sx,
    0, sx, cx);

  Mat R_y = (Mat_<double>(3, 3) << 
    cy, 0, sy,
    0, 1, 0,
    -sy, 0, cy);

  Mat R_z = (Mat_<double>(3, 3) << 
    cz, -sz, 0,
    sz, cz, 0,
    0, 0, 1);

  return R_z * R_y * R_x;
}

