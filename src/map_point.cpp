#include "map_point.h"

#include <opencv2/core.hpp>

#include "matcher.h"
#include "misc.h"
int getMapPointID()
{
  static int _MAP_POINT_ID = -1;
  return ++_MAP_POINT_ID;
}

MapPoint::MapPoint() : id(getMapPointID()) {}
// MapPoint::MapPoint(PosD position, cv::Vec<uint8_t, 32> descriptor)
//     : id(getMapPointID()), position(position), descriptor(descriptor)
// {
// }
MapPoint::MapPoint(
    cv::Mat pos, cv::Vec<uint8_t, 32> descriptor, KeyFrame* refKeyFrame)
    : MapPoint()
{
  this->mPos = pos;
  this->mDescriptor = descriptor;
  this->mRefKeyFrame = refKeyFrame;
}
MapPoint::~MapPoint() {}
int MapPoint::getID() { return this->id; }
cv::Mat MapPoint::getPos() { return this->mPos; }
void MapPoint::setPos(const cv::Mat& pos) { pos.copyTo(pos); }
Eigen::Matrix<double, 3, 1> MapPoint::getPosEigen()
{
  return Eigen::Matrix<double, 3, 1>(
      this->mPos.at<double>(0),
      this->mPos.at<double>(1),
      this->mPos.at<double>(2));
}
cv::Vec<uint8_t, 32> MapPoint::getDescriptor() { return this->mDescriptor; }
std::map<KeyFrame*, uvIdx> MapPoint::getObservations()
{
  return this->mObservations;
}
void MapPoint::applyDepthScale(double depthScale) { this->mPos *= depthScale; }
void MapPoint::addObservation(KeyFrame* keyFrame, uvIdx imgPointIdx)
{
  this->mObservations[keyFrame] = imgPointIdx;
}
void MapPoint::updateDescriptor()
{
  //  obvervations들의 상호 descriptor의 거리를 구한 후 median에 해당하는
  //  descriptor로 설정한다.
  std::vector<cv::Mat> vDescriptors;
  std::map<KeyFrame*, size_t> mObservations;

  // {
  //   unique_lock<mutex> lock1(mMutexFeatures);
  //   if (mbBad) return;
  mObservations = this->mObservations;
  // }

  if (mObservations.empty())
  {
    return;
  }
  vDescriptors.reserve(mObservations.size());
  // for (std::map<KeyFrame*, size_t>::iterator mit = mObservations.begin(),
  //                                            mend = mObservations.end();
  //      mit != mend;
  //      mit++)
  for (auto& [pKF, uvIdx] : mObservations)
  {
    // KeyFrame* pKF = mit->first;

    // if (!pKF->isBad())
    // vDescriptors.push_back(pKF->getDescriptors().row(mit->second));
    vDescriptors.push_back(pKF->getDescriptors().row(uvIdx));
  }

  if (vDescriptors.empty()) return;

  // Compute distances between them
  const size_t N = vDescriptors.size();

  float Distances[N][N];          // if N is 5.
  for (size_t i = 0; i < N; i++)  // 0 ~ 4
  {
    Distances[i][i] = 0;
    for (size_t j = i + 1; j < N; j++)  // [1, 2, 3] ~ 4
    {
      int distij =
          Matcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
      Distances[i][j] = distij;
      Distances[j][i] = distij;
      // 모든 descriptor 사이의 거리를 구한다.
    }
  }

  // Take the descriptor with least median distance to the rest
  // obervation된 frame들의 descriptor들 사이에 거리를 구해서 median 값 중에
  // 가장 작은 median 값을 이 map point의 대표 descriptor로 지정한다.
  int BestMedian = INT_MAX;
  int BestIdx = 0;
  for (size_t i = 0; i < N; i++)
  {
    std::vector<int> vDists(Distances[i], Distances[i] + N);
    // Distances[i] 번째부터 Distances[i] + N 까지의 데이터를 복사
    // 즉, 첫 번째 기술자와 나머지 기술자들
    // it has scala.
    sort(vDists.begin(), vDists.end());
    int median = vDists[0.5 * (N - 1)];
    // Find median value with sorting, (9=4 10=5 11=5)

    if (median < BestMedian)
    {
      BestMedian = median;
      BestIdx = i;
    }
  }

  // {
  // unique_lock<mutex> lock(mMutexFeatures);
  mDescriptor = vDescriptors[BestIdx].clone();
  // }
}
void MapPoint::updateNormalAndDepth()
{
  std::map<KeyFrame*, size_t> observations;
  KeyFrame* refKF;  // 처음으로 발견된 프레임의 주소
  cv::Mat pos;
  {
    // unique_lock<mutex> lock1(mMutexFeatures);
    // unique_lock<mutex> lock2(mMutexPos);
    // if (mbBad) return;
    observations = mObservations;
    refKF = mRefKeyFrame;
    pos = mPos.clone();
  }
  if (observations.empty())
  {
    return;
  }
  cv::Mat normal = cv::Mat::zeros(3, 1, CV_64F);
  int n = 0;
  for (std::map<KeyFrame*, uvIdx>::iterator it = observations.begin(),
                                            end = observations.end();
       it != end;
       ++it)
  {
    KeyFrame* observedKeyFrame = it->first;
    cv::Mat Ow = observedKeyFrame->getCameraCenter();
    cv::Mat vecFromMpToCam = pos - Ow;
    normal += vecFromMpToCam / cv::norm(vecFromMpToCam);
    n++;
  }

  cv::Mat PC = pos - refKF->getCameraCenter();
  const float dist = cv::norm(PC);
  const int level = refKF->getKps()[observations[refKF]].octave;
  {
    // unique_lock<mutex> lock3(mMutexPos);
    // mMaxDistance = dist * KeyFrame::scaleFactors[level];
    mMaxDistance = dist * refKF->scaleFactors[level];
    // mMinDistance = mMaxDistance / KeyFrame::scaleFactors[KeyFrame::nLevels -
    // 1];
    mMinDistance = mMaxDistance / refKF->scaleFactors[KeyFrame::nLevels - 1];
    mNormalVector = normal / n;
  }
}