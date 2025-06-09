#pragma once

#include <cstddef>
#include <opencv2/core/types.hpp>
#include <unordered_map>

#include "frame.h"
#include "map_point.h"
#include "settings.h"

class Camera;

class Map
{
 private:
  int chiThreshold;
  int scale_factor;
  int scale_consistency_factor;
  int cos_max_parallax;

  std::unordered_map<KeyFrameID, KeyFramePtr> keyFrames;
  std::unordered_map<MapPointID, MapPointPtr> mapPoints;

 public:
  Map() {}
  Map(Settings settings)
      : chiThreshold(settings.chi_square_threshold),
        scale_factor(settings.scaleFactor),
        scale_consistency_factor(settings.scale_consistency_factor),
        cos_max_parallax(settings.cos_max_parallax)
  {
  }

  inline void addkeyFrame(KeyFramePtr keyFrame)
  {
    // this->keyFrames.push_back(keyFrame);
    this->keyFrames[keyFrame->getID()] = keyFrame;
  }
  inline void addMapPoint(MapPointPtr mapPoint)
  {
    // this->mapPoints.push_back(map_point);
    this->mapPoints[mapPoint->getID()] = mapPoint;
  };
  inline std::unordered_map<KeyFrameID, KeyFramePtr> getKeyFrames() const
  {
    return this->keyFrames;
  };
  inline std::unordered_map<MapPointID, MapPointPtr> getMapPoints() const
  {
    return this->mapPoints;
  };

  void add_map_points(
      const KeyFrame *const frame1,
      const KeyFrame *const frame2,
      const Camera &camera,
      const cv::Mat &idx_frame1,
      const cv::Mat &idx_frame2,
      const cv::Mat &points4d);
};