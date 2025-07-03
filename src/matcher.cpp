#include "matcher.h"

#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

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

  return dist;  // 해밍 거리는 비트간이 차이이기 때문에 최소 0에서 최대 256
                // 비트까지를 표현한다.
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
