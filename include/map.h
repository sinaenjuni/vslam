#pragma once

#include <opencv2/core/types.hpp>
#include <set>
#include <vector>

#include "map_point.h"
#include "settings.h"

class KeyFrame;

class Map
{
 private:
  int chiThreshold;
  int scale_factor;
  int scale_consistency_factor;
  int cos_max_parallax;

  // std::unordered_map<KeyFrameID, KeyFrame*> keyFrames;
  std::set<KeyFrame*> keyFrames;
  // std::unordered_map<MapPointID, MapPoint*> mapPoints;
  std::set<MapPoint*> mapPoints;
  std::mutex mutexSetterGetter;

 public:
  Map();
  Map(Settings settings);
  ~Map();
  void addKeyFrame(KeyFrame* pKeyFrame);
  void addMapPoint(MapPoint* mapPoint);
  std::vector<KeyFrame*> getAllKFs();
  std::vector<MapPoint*> getAllMPs();

  // inline std::unordered_map<KeyFrameID, KeyFrame*> getKeyFrames() const
  // {
  //   return this->keyFrames;
  // };
  // inline std::unordered_map<MapPointID, MapPoint*> getMapPoints() const
  // {
  //   return this->mapPoints;
  // };

  // void add_map_points(
  //     const KeyFramePtrconst frame1,
  //     const KeyFramePtrconst frame2,
  //     const Camera &camera,
  //     const cv::Mat &idx_frame1,
  //     const cv::Mat &idx_frame2,
  //     const cv::Mat &points4d);
};