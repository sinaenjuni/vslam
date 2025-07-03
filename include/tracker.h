#pragma once

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>

#include "frame.h"
#include "settings.h"

class Matcher;
class Map;
class Tracker;

class TrackerState
{
 protected:
  Tracker *tracker_;

 public:
  virtual ~TrackerState() {};
  void setContext(Tracker *system);
  virtual void process(KeyFrame *pFrame) = 0;
};
class Tracker
{
 private:
  TrackerState *state_;

 public:
  Settings settings;
  cv::VideoCapture cap;
  cv::Ptr<cv::ORB> featureExtractor;
  OrbVocabulary *mpORBVocabulary;
  KeyFrame *lastFrame;
  Matcher *mpMatcher;
  Map *mpMap;

  Tracker(TrackerState *state, Matcher *pMatcher, Map *pMap);
  Tracker(
      TrackerState *state,
      const Settings &settings,
      cv::Ptr<cv::ORB> featureExtractor,
      OrbVocabulary *pORBVocabulary);
  ~Tracker();
  void transitionTo(TrackerState *state);
  void process(KeyFrame *pFrame);
};

class MonocularInitState : public TrackerState
{
 public:
  MonocularInitState() {}
  void process(KeyFrame *pFrame) override;
};
class TrackReferenceKeyFrame : public TrackerState
{
 public:
  TrackReferenceKeyFrame() {}
  void process(KeyFrame *pFrame) override;
};