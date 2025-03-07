#pragma once

#include <opencv2/core/mat.hpp>
#include <queue>

#include "key_frame.h"

class Settings;
class IFeature_matcher;

class Tracker
{
 private:
  IFeature_matcher *matcher;

  // uint max_size;
  // std::queue<Key_frame *> tracked_frames;
  KeyFramePtr lastKeyFrame;

 public:
  Tracker();
  Tracker(Settings settings, IFeature_matcher *matcher);
  Rt track(KeyFramePtr keyFrame, const cv::Mat &view_img);
};
