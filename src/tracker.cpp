#include "tracker.h"

#include <cstddef>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

#include "frame.h"
#include "map.h"
#include "matcher.h"
#include "misc.h"
#include "optimization.h"

void TrackerState::setContext(Tracker *system) { this->tracker_ = system; }
Tracker::Tracker(TrackerState *state, Matcher *pMatcher, Map *pMap)
    : state_(nullptr), lastFrame(nullptr), mpMatcher(pMatcher), mpMap(pMap)
{
  this->transitionTo(state);
}
Tracker::Tracker(
    TrackerState *state,
    const Settings &settings,
    cv::Ptr<cv::ORB> featureExtractor,
    OrbVocabulary *pORBVocabulary)
    : state_(nullptr),
      settings(settings),
      cap(cv::VideoCapture(settings.imgPath)),
      featureExtractor(featureExtractor),
      mpORBVocabulary(pORBVocabulary),
      lastFrame(nullptr)
{
  this->transitionTo(state);
}
Tracker::~Tracker() { delete state_; }
void Tracker::transitionTo(TrackerState *state)
{
  std::cout << "Context: Transition to " << typeid(*state).name() << ".\n";
  if (this->state_ != nullptr) delete this->state_;
  this->state_ = state;
  this->state_->setContext(this);
}
void Tracker::process(KeyFrame *pframe)
{
  // while (true)
  // {
  bool isNextFrame = this->state_->process(pframe);
  if (isNextFrame)
  {
    // break;
  }
  // else
  // {
  //   continue;
  // }
  // }

  return;
}
void Tracker::calcVelocity(KeyFrame *pFK)
{
  mVelocity = pFK->getPose() * lastFrame->getPoseInv();
  // (velocity) = (월드 -> 현재 포즈) @ (이전 포즈 -> 월드)
  // velocity는 이전 프레임에서 바라본 현재 프레임의 위치를 의미한다.
  // 만약 부산에서 바라본 대구의 위치를 알고싶다면, 서울을 기준이라고 봤을때, 각
  // 위치는 서울->부산, 서울->대구가 된다. 따라서 부산->대구는 부산->서울->대구
  // 거쳐서 봐야한다. 이를 변환행렬로 정리하면, (서울->대구) @ (서울->부산)^-1
  // 이 된다.
  float decayingFactor = 0.6;
  mVelocity.at<double>(0, 3) *= decayingFactor;
  mVelocity.at<double>(1, 3) *= decayingFactor;
  mVelocity.at<double>(2, 3) *= decayingFactor;
}
cv::Mat Tracker::getNextPoseWithVelocity()
{
  return mVelocity * lastFrame->getPose();
}

bool MonocularInitState::process(KeyFrame *pFrame)
{
  if (tracker_->lastFrame == nullptr)
  {
    tracker_->lastFrame = pFrame;
    return true;
  }
  std::vector<int> match1, match2;
  int nMatches = tracker_->mpMatcher->matchInGrid(
      tracker_->lastFrame, pFrame, match1, match2);
  PRINT("nMatches", nMatches);

  if (nMatches < 100)
  {
    PRINT("Not enough matches, skipping frame.");
    // delete tracker_->lastFrame;
    tracker_->lastFrame = pFrame;
    return true;
  }

  std::vector<cv::Point2f> points1, points2;
  for (size_t i = 0; i < match2.size(); ++i)
  {
    if (match2[i] >= 0)
    {
      points1.push_back(tracker_->lastFrame->getKps()[match2[i]].pt);
      points2.push_back(pFrame->getKps()[i].pt);
    }
  }

  cv::Mat Rcw, tcw;
  std::vector<uchar> inliersF;
  cv::Mat F = cv::findFundamentalMat(
      points1, points2, cv::FM_RANSAC, 3.0, 0.99, inliersF);
  cv::Mat E = pFrame->K.t() * F * pFrame->K;
  cv::recoverPose(E, points1, points2, pFrame->K, Rcw, tcw, inliersF);

  // The matrix R is the rotation matrix that transforms from wolrd to camera.
  std::vector<cv::Point2d> inlierPoints1, inlierPoints2;
  for (size_t i = 0, c = 0; i < match2.size(); ++i)
  {
    if (match2[i] >= 0 && inliersF[c++])
    {
      inlierPoints1.push_back(tracker_->lastFrame->getKps()[match2[i]].pt);
      inlierPoints2.push_back(pFrame->getKps()[i].pt);
    }
    else
    {
      match2[i] = -1;
    }
  }

  tracker_->lastFrame->setPose(cv::Mat::eye(4, 4, CV_64F));
  pFrame->setPose(Rcw, tcw);

  cv::Mat points4D;
  cv::triangulatePoints(
      tracker_->lastFrame->getP(),
      pFrame->getP(),
      inlierPoints1,
      inlierPoints2,
      points4D);
  for (size_t i = 0, c = 0; i < match2.size(); ++i)
  {
    if (match2[i] >= 0)
    {
      cv::Mat X = points4D.col(c++);
      X /= X.at<double>(3);

      MapPoint *mp = new MapPoint(
          cv::Mat(X.rowRange(0, 3)), pFrame->getDescriptors().row(i), pFrame);

      mp->addObservation(tracker_->lastFrame, match2[i]);
      mp->addObservation(pFrame, i);
      mp->updateDescriptor();
      mp->updateNormalAndDepth();

      tracker_->lastFrame->addMapPoint(mp, match2[i]);
      pFrame->addMapPoint(mp, i);
      tracker_->mpMap->addMapPoint(mp);
    }
  }

  tracker_->lastFrame->computeBoW();
  pFrame->computeBoW();

  tracker_->mpMap->addKeyFrame(tracker_->lastFrame);
  tracker_->mpMap->addKeyFrame(pFrame);
  tracker_->lastFrame->updateConnections();
  pFrame->updateConnections();

  Optimizer::GlobalBundleAdjustment(tracker_->mpMap, 20);
  tracker_->calcVelocity(pFrame);
  tracker_->transitionTo(new TrackReferenceKeyFrame);
  return true;
}

bool TrackReferenceKeyFrame::process(KeyFrame *pFrame)
{
  // pFrame->setPose(tracker_->getNextPoseWithVelocity());
  // tracker_->mpMap->addKeyFrame(pFrame);
  // tracker_->lastFrame = pFrame;

  // pFrame->computeBoW();
  // tracker_->mpMatcher->set(0.7, true);

  // std::vector<MapPoint *> vMatchedMPs;
  // int nMatches =
  //     tracker_->mpMatcher->matchInBow(tracker_->lastFrame, pFrame,
  //     vMatchedMPs);
  // PRINT(nMatches);
  // if (nMatches < 15)
  // {
  // }
  return false;
}