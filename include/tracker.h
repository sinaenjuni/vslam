#pragma once

#include <cstddef>
#include <memory>

#include "frame.h"

class Tracker;
class TrackerState
{
 public:
  virtual void processFrame(Tracker& tracker, const KeyFrame* frame) = 0;
  // pure vitual function.
  virtual ~TrackerState() = default;
  // delegate to compiler to make a destructor.
};

class Tracker
{
 private:
  std::unique_ptr<TrackerState> state;
  KeyFrame* lastFrame = nullptr;

 public:
  Tracker(TrackerState* initState) : state(initState) {}
  KeyFrame* getLastFrame() { return lastFrame; }
  void setLastFrame(KeyFrame* lastFrame) { lastFrame = lastFrame; }
  void setState(TrackerState* trackerState) { state.reset(trackerState); }
  void processFrame(KeyFrame* frame) { state->processFrame(*this, frame); }
};

class InitTrackingState : public TrackerState
{
 public:
  void processFrame(Tracker& tracker, const KeyFrame* frame) override
  {
    tracker.getLastFrame();
  }
};