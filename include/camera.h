#pragma once

#include <opencv2/opencv.hpp>

#include "settings.h"

class Camera
{
 private:
  int width;
  int height;
  double fx, fy, cx, cy, bf;  // bf is base line multiply focal length
  cv::Mat K;                  // Intrinsic matrix
  cv::Mat Kinv;               // Inverse of intrinsic matrix
  cv::Mat Rt;                 // Extrinsic matrix

 public:
  Camera();
  Camera(Settings &settings);
  Camera(
      double fx,
      double fy,
      double cx,
      double cy,
      double bf,
      int width,
      int height);
  ~Camera();

  inline cv::Mat get_K() { return this->K; };

  void unproject(const cv::Mat &kps, cv::Mat &kpsn) const;
  void project(const cv::Mat &kpsn, cv::Mat &kps) const;
};