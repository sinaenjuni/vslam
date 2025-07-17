#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Dense>

#include "frame.h"

class KeyFrame;
class MapPoint;

class Converter
{
 public:
  static g2o::SE3Quat ToSE3Quat(const cv::Mat &cvMat);
  static Eigen::Matrix<double, 3, 1> ToVector3d(const cv::Mat &pos);
  static cv::Mat ToCvMat(const g2o::SE3Quat SE3);
  static cv::Mat ToCvMat(const Eigen::Matrix<double, 4, 4> &m);
  static cv::Mat ToCvMat(const Eigen::Matrix<double, 3, 1> &m);
};
class Map;
class Optimizer
{
 public:
  static void GlobalBundleAdjustment(Map *pMap, uint iterators);
  static void BundleAdjustment(
      const std::vector<KeyFrame *> &keyFrames,
      const std::vector<MapPoint *> &mapPoints,
      int nIterations);
};
