#pragma once

#include <Eigen/Dense>
#include <map>
#include <opencv2/core/mat.hpp>
// #include "entities.h"

using uvIdx = size_t;
using uvIdx = size_t;
class KeyFrame;

class MapPoint
{
 private:
  int id;
  cv::Mat mPos;
  KeyFrame* mRefKeyFrame;  // reference keyframe
  // std::array<uint16_t, 32> descriptor;
  // cv::Vec<uint8_t, 32> mDescriptor;
  cv::Mat mDescriptor;
  // std::set<KeyFrameID> observed_by;
  std::map<KeyFrame*, uvIdx> mObservations;

  // Normal vector and depth information
  float mMinDistance;
  float mMaxDistance;
  cv::Mat mNormalVector;

 public:
  MapPoint();
  MapPoint(
      cv::Mat position, cv::Vec<uint8_t, 32> descriptor, KeyFrame* refKeyFrame);
  ~MapPoint();
  int getID();
  cv::Mat getPos();
  void setPos(const cv::Mat& pos);
  Eigen::Matrix<double, 3, 1> getPosEigen();
  cv::Vec<uint8_t, 32> getDescriptor();
  std::map<KeyFrame*, uvIdx> getObservations();
  void applyDepthScale(double depthScale);
  void addObservation(KeyFrame* keyFrame, uvIdx imgPointIdx);
  void updateDescriptor();
  void updateNormalAndDepth();

  // inline void addObservation(KeyFrameID keyFrameID) {
  // observations.insert(keyFrameID); };

  // inline std::set<KeyFrameID> getObservation() { return observed_by; };
  // inline void removeObservation(KeyFrameID keyPointID) {
  // observations.erase(keyPointID); };
};
