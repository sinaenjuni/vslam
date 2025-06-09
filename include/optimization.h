#include <g2o/types/slam3d/se3quat.h>

#include <Eigen/Dense>

#include "entities.h"

class Converter
{
 public:
  static g2o::SE3Quat toSE3Quat(const cv::Mat &cvMat);
  static Eigen::Matrix<double, 3, 1> toVector3d(const PosD &pos);
};

class Optimizer
{
 public:
  static void bundleAdjustment(
      const std::vector<KeyFramePtr> &keyFrames,
      const std::vector<MapPointPtr> &mapPoints,
      int nIterations);
};
