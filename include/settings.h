#pragma once
#include <iostream>
#include <vector>

struct Settings {
  std::string sensor_type;
  // camera information
  int width;
  int height;
  float fx;
  float fy;
  float cx;
  float cy;
  double bf;

  // image and time data
  std::string img_path;
  std::string imgr_path;
  std::string timestamp_path;
  std::vector<float> timestamp;

  // feature
  int n_levels;
  int n_points;
  int feature_size;
  double scale_factor;
  std::vector<double> scale2_factors;
  std::vector<double> scale2inv_factors;

  // optimization
  double kChi2Mono;
  double kChi2Stereo;
  double cos_max_parallax;
  double scale_consistency_factor;
  int desired_median_depth;

  void print() const;
};

Settings prase_settings(const std::string &yaml_file_path);