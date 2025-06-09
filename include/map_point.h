#pragma once
// #include <set>
#include <map>

#include "entities.h"

class MapPoint
{
 private:
  int id;
  PosD position;
  // std::array<uint16_t, 32> descriptor;
  cv::Vec<uint8_t, 32> descriptor;
  // std::set<KeyFrameID> observed_by;
  std::map<KeyFramePtr, ImgPointIdx> observations;

 public:
  MapPoint();
  MapPoint(PosD position, cv::Vec<uint8_t, 32> descriptor);
  ~MapPoint();

  inline int getID() { return id; };
  inline PosD getPos() { return position; };
  inline void applyDepthScale(double depthScale)
  {
    position.x *= depthScale;
    position.y *= depthScale;
    position.z *= depthScale;
  }
  inline cv::Vec<uint8_t, 32> getDescriptor() { return descriptor; };
  // inline void addObservation(KeyFrameID keyFrameID) { observations.insert(keyFrameID); };
  inline void addObservation(KeyFramePtr keyFrame, ImgPointIdx imgPointIdx)
  {
    observations[keyFrame] = imgPointIdx;
  };
  // inline std::set<KeyFrameID> getObservation() { return observed_by; };
  inline std::map<KeyFramePtr, ImgPointIdx> getObservations() { return this->observations; };
  // inline void removeObservation(KeyFrameID keyPointID) { observations.erase(keyPointID); };
};
