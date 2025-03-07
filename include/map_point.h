#pragma once
#include <set>

#include "entities.h"

class MapPoint
{
 private:
  int id;
  PosD position;
  // std::array<uint16_t, 32> descriptor;
  cv::Vec<uint8_t, 32> descriptor;
  std::set<KeyFrameID> observed_by;

 public:
  MapPoint();
  MapPoint(PosD position, cv::Vec<uint8_t, 32> descriptor);
  ~MapPoint();

  inline int getID() { return id; };
  inline PosD getPos() { return position; };
  inline cv::Vec<uint8_t, 32> getDescriptor() { return descriptor; };
  inline void addObservation(KeyFrameID keyFrameID)
  {
    observed_by.insert(keyFrameID);
  };
  inline std::set<KeyFrameID> getObservation() { return observed_by; };
  inline void removeObservation(KeyFrameID keyPointID)
  {
    observed_by.erase(keyPointID);
  };
};
