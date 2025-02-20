#pragma once

#include <opencv2/opencv.hpp>

class Camera
{
 private:
  int width, height;
  double fx, fy, cx, cy, bf;  // bf is base line multiply focal length
  cv::Mat K;                  // Intrinsic matrix
  cv::Mat Kinv;               // Inverse of intrinsic matrix
  cv::Mat Rt;                 // Extrinsic matrix

 public:
  Camera();
  Camera(
      double fx,
      double fy,
      double cx,
      double cy,
      double bf,
      int width,
      int height);
  ~Camera();

  void unproject(const cv::Mat &kps, cv::Mat &kpsn);
  void project(const cv::Mat &kpsn, cv::Mat &kps, cv::Mat &depth);
};