#pragma once

#include <opencv2/core/mat.hpp>
#include <queue>

class Rt;
class Key_frame;
class Settings;
class IFeature_matcher;

class Tracker
{
 private:
  IFeature_matcher *matcher;

  uint max_size;
  std::queue<Key_frame *> tracked_frames;

 public:
  Tracker();
  Tracker(Settings settings, IFeature_matcher *matcher);
  Rt track(Key_frame *frame, const cv::Mat &view_img);
};
