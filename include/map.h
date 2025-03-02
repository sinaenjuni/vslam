#pragma once

#include "key_frame.h"
#include "map_point.h"
#include "settings.h"

class Map
{
 private:
  int kChi;
  int scale_factor;
  int scale_consistency_factor;
  int cos_max_parallax;

  std::vector<Key_frame *> key_frames;
  std::vector<Map_point *> map_points;

 public:
  Map() {}
  Map(Settings settings)
      : kChi(settings.kChi2Mono),
        scale_factor(settings.scale_factor),
        scale_consistency_factor(settings.scale_consistency_factor),
        cos_max_parallax(settings.cos_max_parallax)
  {
  }

  inline void add_key_frame(Key_frame *key_frame) { this->key_frames.push_back(key_frame); }
  inline void add_map_point(Map_point *map_point) { this->map_points.push_back(map_point); };
  inline std::vector<Key_frame *> get_key_frames() const { return this->key_frames; };
  inline std::vector<Map_point *> get_map_points() const { return this->map_points; };

  void add_map_points(
      const Key_frame *const frame1,
      const Key_frame *const frame2,
      const Camera &camera,
      const cv::Mat &idx_frame1,
      const cv::Mat &idx_frame2,
      const cv::Mat &points4d);
};