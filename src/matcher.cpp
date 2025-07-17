#include "matcher.h"

#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#include "FeatureVector.h"
#include "frame.h"

constexpr int TH_HIGH = 100;
constexpr int TH_LOW = 50;
constexpr int HISTO_LENGTH = 30;
constexpr float HISTO_FACTOR = 1.0f / static_cast<float>(HISTO_LENGTH);

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
// ORB desciptor is 256bit / 8bit(1byte) = 32bytes,
// 4bytes = 32bit
// 그러니까 이 알고리즘은 32비트에서 동작하도록 설계되었으며,
// 2비트, 4비트, 8비트 단위로 묶어서 비트의 개수를 in-place 형태로 셀 수 있도록
// 설계된 알고리즘이다.
// 간단하게 예를 들면
// v가 11001111 라면,
// 11 00 11 11 형태의 2비트 단위로 쪼갤 수 있다.
// 10 00 10 10 (1이 2개면 2를 나타내느 10, 1이 1개면 01, 1이 0개면 00)
// 이런 알고리즘을 구현하기 위해

//  00 01 10 11 (>>1)
//  00 00 01 01
//  01 01 01 01 (&)
//  00 00 01 01
// (이 과정에서 00 01 10 11이 00 00 01 01이 된다.)
// 상위 비트가 있는 경우에는 1이 나타나는 것이다.

//  00 01 10 11
//  00 00 01 01 (-)
//  00 01 01 10 ==> 0개 1개 1개 1개로 해석이 가능
// 이렇게 되면 상위 비트만 있는 경우에도 1개로 취급할 수 있다.

//  0001 0010
//  0011 0011
//  0001 0010

// 4비트 단위로 개수를 파악한다.
// 0000 0010 0011 0100 0101 0110 0111 1000 1001 1010 1011 1100 1101 1110 1111
// 0000 0001 0010 0001 0010 0010 0011 0001 0010 0010 0011 0011 0011 0011 0100

// 하위 2비트를 없애 상위 2비트만 확인한다.
// 0000 0000 0000 0001 0001 0001 0001 0010 0010 0010 0010 0011 0011 0011 0011
// (>>2) 0011 0011 0011 0011 0011 0011 0011 0011 0011 0011 0011 0011 0011 0011
// 0011

// 00 01 10 11
// 00 11 00 11 (&)
// 00 01 00 11

//  00 01 01 10 (>>2)
//  00 00 01 01
//  00 11 00 11 (&)
//  00 00 00 01

int Matcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++)
  {
    unsigned int v = *pa ^ *pb;       // 1000
    v = v - ((v >> 1) & 0x55555555);  // 1000 - (0100 & 0101)
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
  // The hamming distance is in [0, 256] range.
}

void Matcher::computeThreeMaxima(
    std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3)
{
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++)
  {
    const int s = histo[i].size();  // 30개로 나눠져 있는 histogram의 각 크기
    if (s > max1)
    {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    }
    else if (s > max2)
    {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    }
    else if (s > max3)
    {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1)
  // max2가 max1의 10%보다 작으면, -1 할당
  {
    ind2 = -1;
    ind3 = -1;
  }
  else if (max3 < 0.1f * (float)max1)
  // max3가 max1의 10%보다 작으면, -1 할당
  {
    ind3 = -1;
  }
}

Matcher::Matcher(float testRatio, bool checkOrientation)
    : mfTestRatio(testRatio),
      mbCheckOrientation(checkOrientation),
      mMatcher(new cv::BFMatcher())
{
}
Matcher::~Matcher() {}
void Matcher::set(float testRatio, bool checkOrientation)
{
  mfTestRatio = testRatio;
  mbCheckOrientation = checkOrientation;
}

// int Matcher::knnMatch(
//     KeyFrame *F1,  // train
//     KeyFrame *F2,  // query
//     std::vector<int> &matchF1,
//     std::vector<int> &matchF2)
// {
//   std::vector<std::vector<cv::DMatch>> matches;
//   matcher->knnMatch(F2->getDescriptors(), F1->getDescriptors(), matches, 2);

//   matchF1 = std::vector<int>(F1->getN(), -1);
//   matchF2 = std::vector<int>(F2->getN(), -1);
//   std::unordered_map<int, std::pair<int, float>> duplications(F2->getN());
//   // pair index, distance
//   // 현재 영상에 존재하는 포인트를 기준으로 이전 영상에서 유사한 포인트를
//   // 찾는다. 이때, 현재 영상에 존재하는 포인트는 이전 영상에 존재하는 포인트
//   // 여러개(K개)에 연결될 수 있다.

//   int nMatches = 0;
//   for (const std::vector<cv::DMatch> &match : matches)
//   {
//     // A m0 is a nearest neighbor and a m1 is a next ones.
//     // Find two nearest points based on a query descriptor.
//     const cv::DMatch &m0 = match[0];
//     const cv::DMatch &m1 = match[1];

//     if (m1.distance * 0.75 < m0.distance)
//     {
//       // The ratio test, remove points that are not
//       // sufficiently close to the nearest neighbor.
//       continue;
//     }

//     if (duplications.find(m0.trainIdx) == duplications.end())
//     // trainIdx가 중복되지 않는경우
//     {
//       matchF1[m0.trainIdx] = m0.queryIdx;
//       matchF2[m0.queryIdx] = m0.trainIdx;
//       duplications[m0.trainIdx] = {m0.queryIdx, m0.distance};
//       nMatches++;
//     }
//     else  // trainIdx가 중복되는 경우
//     {
//       if (m0.distance < duplications[m0.trainIdx].second)
//       // 이전 trainIdx 보다 해밍 거리가 가까우면 대체
//       {
//         int preQueryIdx = duplications[m0.trainIdx].first;
//         matchF1[m0.trainIdx] = m0.queryIdx;
//         matchF2[preQueryIdx] = m0.trainIdx;
//         duplications[m0.trainIdx] = {m0.queryIdx, m0.distance};
//       }
//     }
//   }
//   return nMatches;
// }

int Matcher::matchInGrid(
    KeyFrame *F1,  // train
    KeyFrame *F2,  // query
    std::vector<int> &matchF1,
    std::vector<int> &matchF2)
{
  int nMatches = 0;
  matchF1 = std::vector<int>(F1->getN(), -1);
  matchF2 = std::vector<int>(F2->getN(), -1);
  std::vector<int> vDistances(F2->getN(), INT_MAX);

  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
  {
    rotHist[i].reserve(500);
  }

  for (size_t i1 = 0; i1 < F1->getN(); ++i1)
  {
    const cv::KeyPoint kp1 = F1->getKps()[i1];
    int level1 = kp1.octave;
    if (level1 > 0)
    {
      continue;
    }

    cv::Mat d1 = F1->getDescriptors().row(i1);

    int bestDist = INT_MAX;
    int secondBestDist = INT_MAX;
    int bestIdx = -1;

    std::vector<size_t> vIndInGrid =
        F2->getKpsInGrid(kp1.pt.x, kp1.pt.y, 100, level1, level1);
    if (vIndInGrid.empty())
    {
      continue;
    }

    for (const size_t &i2 : vIndInGrid)
    {
      cv::Mat d2 = F2->getDescriptors().row(i2);
      int dist = DescriptorDistance(d1, d2);
      if (vDistances[i2] <= dist)
      {
        continue;  // 현재 프레임을 기준으로 kp1과 kp2의 거리
      }
      if (dist < bestDist)
      {
        secondBestDist = bestDist;
        bestDist = dist;
        bestIdx = i2;  // 현재 프레임의 kp의 index
      }
      else if (dist < secondBestDist)
      {
        secondBestDist = dist;
      }
    }

    // for (size_t i2 = 0; i2 < F2->getN(); ++i2)
    // {
    //   const cv::KeyPoint kp2 = F2->getKps()[i2];
    //   int level2 = kp2.octave;
    //   if (level2 > 0)
    //   {
    //     continue;
    //   }
    //   cv::Mat d2 = F2->getDescriptors().row(i2);
    //   int dist = descriptorDistance(d1, d2);
    //   if (vDistances[i2] <= dist)
    //   {
    //     continue;
    //   }
    //   if (dist < bestDist)
    //   {
    //     secondBestDist = bestDist;
    //     bestDist = dist;
    //     bestIdx = i2;  // 현재 프레임의 kp의 index
    //   }
    //   else if (dist < secondBestDist)
    //   {
    //     secondBestDist = dist;
    //   }
    // }

    if (bestDist <= TH_HIGH)
    {
      if (bestDist < (float)secondBestDist * mfTestRatio)
      {
        if (matchF2[bestIdx] >= 0)
        {
          matchF1[matchF2[bestIdx]] = -1;
          --nMatches;
        }
        matchF1[i1] = bestIdx;  // idxPrev = idxCur
        matchF2[bestIdx] = i1;  // idxCur = idxPrev
        vDistances[bestIdx] = bestDist;
        nMatches++;
      }

      if (mbCheckOrientation)
      {
        float rot = F1->getKps()[i1].angle - F2->getKps()[bestIdx].angle;
        if (rot < 0.0f)
        {
          rot += 360.0f;  // -20 -> +340와 같이 시계 방향을 기준으로
                          // 통일시키기 위해 360을 더한다.
        }
        int bin = round(rot * HISTO_FACTOR);
        if (bin == HISTO_LENGTH)
        {
          bin = 0;
        }
        assert(bin >= 0 && bin < HISTO_LENGTH);
        rotHist[bin].push_back(i1);
      }
    }
  }

  if (mbCheckOrientation)
  {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++)
    {
      if (i == ind1 || i == ind2 || i == ind3)
      {
        continue;
      }
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
      {
        int idx1 = rotHist[i][j];
        if (matchF1[idx1] >= 0)
        {
          matchF2[matchF1[idx1]] = -1;
          matchF1[idx1] = -1;
          nMatches--;
        }
      }
    }
  }
  return nMatches;
}
int Matcher::matchInBow(
    KeyFrame *pKF, KeyFrame *pF, std::vector<MapPoint *> vMatchedMPs)
{
  const std::vector<MapPoint *> vpMPsKF = pKF->getMapPoints();
  vMatchedMPs =
      std::vector<MapPoint *>(pKF->getN(), static_cast<MapPoint *>(nullptr));
  const DBoW2::FeatureVector &vFeatVecKF = pKF->getFeatVec();
  const DBoW2::FeatureVector &vFeatVecF = pF->getFeatVec();

  int nMatches = 0;
  std::vector<int> rotHist[HISTO_LENGTH];  // 30
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);                 // 500 per indices
  const float factor = 1.0f / HISTO_LENGTH;  // 1/30

  DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
  DBoW2::FeatureVector::const_iterator Fit = vFeatVecF.begin();
  DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
  DBoW2::FeatureVector::const_iterator Fend = vFeatVecF.end();

  while (KFit != KFend && Fit != Fend)
  {
    if (KFit->first == Fit->first)  // BoW vocablury 노드가 같다면,
    // first항목 BoW 트리의 특정 노드 ID (NodeId) = visual word
    {
      const std::vector<unsigned int> vIndicesKF = KFit->second;
      const std::vector<unsigned int> vIndicesF = Fit->second;
      // 같은 vocablury node에 속하는 KP의 indices

      for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
      // KF의 key point 인덱스를 순회
      {
        const unsigned int realIdxKF = vIndicesKF[iKF];
        // 이 부분이 이제 KP의 인덱스가 된다.
        // MapPoints associated to keypoints, NULL pointer if no association.
        MapPoint *pMP = vpMPsKF[realIdxKF];
        // 인덱스에 해당하는 KF의 MP를 가져온다.

        if (!pMP)
        {
          // NULL pointer는 관련없는 다른 프레임과의 관련성이 없는 포인트를 의미
          continue;
        }

        // if (pMP->isBad())
        // {
        //   continue;
        // }

        const cv::Mat &dKF = pKF->getDescriptors().row(realIdxKF);
        // 해당 이미지 포인트가 MP를 가지고 있다면, descriptor를 가져온다.

        int bestDist1 = 256;  // 32bytes의 최대 hamming distance는 256
        int bestIdxF = -1;
        int bestDist2 = 256;

        for (size_t iF = 0; iF < vIndicesF.size(); iF++)
        // F에 존재하는 같은 BoW node의 index를 순회
        {
          const unsigned int realIdxF = vIndicesF[iF];

          if (vMatchedMPs[realIdxF]) continue;

          const cv::Mat &dF = pF->getDescriptors().row(realIdxF);
          // 같은 BoW node에 속하는 F의 orb descriptor를 가져온다.

          const int dist = DescriptorDistance(dKF, dF);
          // hamming distance를 계산

          if (dist < bestDist1)
          {
            bestDist2 = bestDist1;  // best1을 best2로 이동
            bestDist1 = dist;       // best1에 dist 할당
            bestIdxF = realIdxF;    // best1의 KP index 할당(F를 기준)
          }
          else if (dist < bestDist2)
          // best1 보다는 크지만 best2보다는 작은 dist는 best2에 dist 저장
          {
            bestDist2 = dist;
          }
        }

        if (bestDist1 <= TH_LOW)
        {  // mfTestRatio = 0.7, ratio test로 best2에 0.7배 해도 best1보다
           // dist가 가깝다면 이 포인트는 일치하는 포인트라고 간주
          if (static_cast<float>(bestDist1) <
              mfTestRatio * static_cast<float>(bestDist2))
          {
            vMatchedMPs[bestIdxF] = pMP;
            // best1의 index에 대한 MP를 저장

            const cv::KeyPoint &kp = pKF->getKps()[realIdxKF];
            // best1의 index에 대한 KP

            if (mbCheckOrientation)
            {
              float rot = kp.angle - pF->getKps()[bestIdxF].angle;
              // KF과 F의 KP의 각도 차이를 계산
              if (rot < 0.0)  // 회전 일관성 보장을 위해
                rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(bestIdxF);
            }
            nMatches++;
          }
        }
      }

      KFit++;
      Fit++;
    }
    else if (KFit->first < Fit->first)
    // 같은 node가 나올 때까지, 더 낮은 node를 가르키는 iterator에 높은 node를
    // 가르키는 iterator의 위치로 옮긴다. map은 기본적으로 정렬된 상태로
    // 데이터를 기록한다. KFit와 Kit 중에서 더 낮은 node를 가르키고 있다면,
    {
      KFit = vFeatVecKF.lower_bound(Fit->first);
      // lower_bound 의미상, 이상에 해당하는 값을 찾는다.
      // m[10] = "a";
      // m[20] = "b";
      // m[30] = "c";
      // m.lower_bound(15) -> 15는 없지만 m[20]에 해당하는 iterator를 반환한다.
      // m.lower_bound(10) -> 10은 존재하기 때문에 m[10]에 해당하는 iterator를
      // 반환한다. upper_bound 의미상, 초과에 해당하는 값을 찾는다.
      // m.upper_bound(15) -> 15는 없지만 m[20]에 해당하는 iterator를 반환한다.
      // m.upper_bound(10) -> 10은 존재하지만 m[20]에 해당하는 iterator를
      // 반환한다.
    }
    else  // 반대로 F의 node가 작다면,
    {
      Fit = vFeatVecF.lower_bound(KFit->first);
    }
  }

  if (mbCheckOrientation)
  {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++)
    {
      if (i == ind1 || i == ind2 || i == ind3)
      {
        continue;
      }
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
      {
        vMatchedMPs[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
        nMatches--;
      }
    }
  }

  return nMatches;
}