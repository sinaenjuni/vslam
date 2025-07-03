#include "tracker.h"

#include <cstddef>

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
      lastFrame(nullptr),
      settings(settings),
      cap(cv::VideoCapture(settings.imgPath)),
      featureExtractor(featureExtractor),
      mpORBVocabulary(pORBVocabulary)
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
  this->state_->process(pframe);
  return;
}

void MonocularInitState::process(KeyFrame *pFrame)
{
  if (tracker_->lastFrame == nullptr)
  {
    tracker_->lastFrame = pFrame;
    return;
  }
  std::vector<int> match1, match2;  // 2000
  int nMatches = tracker_->mpMatcher->matchInGrid(
      tracker_->lastFrame, pFrame, match1, match2);
  PRINT("nMatches", nMatches);

  if (nMatches < 100)
  {
    PRINT("Not enough matches, skipping frame.");
    // delete tracker_->lastFrame;
    tracker_->lastFrame = pFrame;
    return;
  }

  std::vector<cv::Point2f> points1, points2;  // 200
  for (size_t i = 0; i < match2.size(); ++i)
  {
    if (match2[i] >= 0)
    {
      points1.push_back(tracker_->lastFrame->getKps()[match2[i]].pt);
      points2.push_back(pFrame->getKps()[i].pt);
    }
  }

  cv::Mat Rcw, tcw;
  std::vector<uchar> inliersF;  // 200
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

  tracker_->transitionTo(new TrackReferenceKeyFrame);
  return;
}

void TrackReferenceKeyFrame::process(KeyFrame *pFrame)
{
  pFrame->computeBoW();
  tracker_->mpMatcher->set(0.7, true);
  

}