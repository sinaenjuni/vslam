#pragma once
#include <iostream>
#include <vector>

struct Settings
{
  // 3d viewer settings
  std::string window_name;
  int windowWidth;
  int windowHeight;
  int ViewpointX;
  int ViewpointY;
  float ViewpointZ;
  int ViewpointF;

  // sensor type
  std::string sensor_type;

  // camera information
  int imgWidth;
  int imgHeight;
  float fx;
  float fy;
  float cx;
  float cy;
  float bf;

  float k1;
  float k2;
  float p1;
  float p2;

  // image and time data
  std::string imgPath;
  std::string imgrPath;
  std::string timestampPath;
  std::vector<float> timestamp;

  // image feature
  int nLevels;
  int nPoints;
  int featureSize;
  float scaleFactor;
  std::vector<float> scale2Factors;
  std::vector<float> scale2InvFactors;

  // optimization
  float cos_max_parallax;
  float scale_consistency_factor;
  int desired_median_depth;
  // float kChi2Mono;
  // float kChi2Stereo;
  float chi_square_threshold;
  bool use_verbose;
  bool use_robust_kernel;

  void print() const;
  static Settings prase_settings(const std::string &yaml_file_path);
};
