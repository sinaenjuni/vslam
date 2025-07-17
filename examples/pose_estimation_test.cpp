#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/fast_math.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

#include "extractor.h"
#include "misc.h"
#include "settings.h"

namespace fs = std::filesystem;
constexpr int HISTO_LENGTH = 30;

constexpr int nPoints = 3000;
constexpr int nLevels = 8;
constexpr int borderSize = 19;
// 19 is minimum and pracical value, 31 is safe and conservative
constexpr float scaleFactor = 1.2f;
const std::vector<float> scaleFactors = {
    1.0f, 1.2f, 1.44f, 1.728f, 2.0736f, 2.48832f, 2.985984f, 3.5831808f};
const std::vector<float> scaleFactorsInv = {
    1.0f,
    0.833333f,
    0.694444f,
    0.578703f,
    0.482253f,
    0.401877f,
    0.334897f,
    0.279914f};

void computeThreeMaxima(
    std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3)
{
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++)
  {
    const int s = histo[i].size();
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
  {
    ind2 = -1;
    ind3 = -1;
  }
  else if (max3 < 0.1f * (float)max1)
  {
    ind3 = -1;
  }
}

int matchKps(
    const std::vector<cv::KeyPoint> &kps1,
    const std::vector<cv::KeyPoint> &kps2,
    const cv::Mat &desc1,
    const cv::Mat &desc2,
    std::vector<int> &vMatchIndices)
{
  cv::BFMatcher matcher(cv::NORM_HAMMING, false);
  std::vector<std::vector<cv::DMatch>> matches;
  matcher.knnMatch(desc2, desc1, matches, 2);

  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
  {
    rotHist[i].reserve(500);
  }
  const float histFactor = 1.0f / HISTO_LENGTH;

  vMatchIndices = std::vector<int>(matches.size(), -1);
  std::map<int, std::pair<int, float>> mDistances;
  int nMatches = 0;
  for (const auto &match : matches)
  {
    cv::DMatch m1 = match[0], m2 = match[1];
    if (m1.distance > 0.6f * m2.distance)
    {
      continue;
    }

    if (mDistances.find(m1.trainIdx) == mDistances.end())
    {
      vMatchIndices[m1.queryIdx] = m1.trainIdx;
      mDistances[m1.trainIdx] = std::make_pair(m1.queryIdx, m1.distance);
      ++nMatches;
    }
    else
    {
      if (mDistances[m1.trainIdx].second > m1.distance)
      {
        vMatchIndices[m1.queryIdx] = m1.trainIdx;
        mDistances[m1.trainIdx] = std::make_pair(m1.queryIdx, m1.distance);
      }
    }
  }
  for (size_t i = 0; i < vMatchIndices.size(); ++i)
  {
    if (vMatchIndices[i] == -1)
    {
      continue;
    }
    float angle = kps1[vMatchIndices[i]].angle - kps2[i].angle;
    // The angle is in [0,360) degrees.
    if (angle < 0.0f) angle += 360.0f;
    int bin = std::round(angle * histFactor);
    if (bin == HISTO_LENGTH) bin = 0;
    rotHist[bin].push_back(i);
  }

  int ind1 = -1, ind2 = -1, ind3 = -1;
  computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

  for (int i = 0; i < HISTO_LENGTH; i++)
  {
    if (i == ind1 || i == ind2 || i == ind3) continue;
    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
    {
      int idx = rotHist[i][j];
      if (vMatchIndices[idx] != -1)
      {
        vMatchIndices[rotHist[i][j]] = -1;
        nMatches--;
      }
    }
  }
  return nMatches;
}


int main(int argc, char **argv)
{
  Settings settings =
      Settings::prase_settings("../settings/KITTI_Monocular.yaml");
  settings.print();
  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = settings.fx;
  K.at<float>(1, 1) = settings.fy;
  K.at<float>(0, 2) = settings.cx;
  K.at<float>(1, 2) = settings.cy;

  std::string imgPath = "../data/KITTI/sequences/00/image_0/";
  std::vector<fs::path> fileList;

  FastOrbExtractor featureExtractor(
      4000,
      settings.nLevels,
      settings.imgWidth,
      settings.imgHeight,
      settings.scaleFactor,
      7,
      20);

  try
  {
    fileList.assign(fs::directory_iterator(imgPath), fs::directory_iterator());
  }
  catch (fs::filesystem_error &e)
  {
    std::cerr << "Error accessing directory: " << e.what() << '\n';
    return 1;
  }
  std::sort(
      fileList.begin(),
      fileList.end(),
      [](const fs::path &a, const fs::path &b)
      { return a.filename().string() < b.filename().string(); });

  cv::Mat img1 = cv::imread(fileList[0].string(), cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(fileList[1].string(), cv::IMREAD_GRAYSCALE);
  cv::Mat colorImg;
  cv::cvtColor(img2, colorImg, cv::COLOR_GRAY2BGR);

  std::vector<cv::KeyPoint> kps1, kps2;
  cv::Mat desc1, desc2;
  featureExtractor.detectAndCompute(img1, kps1, desc1);
  featureExtractor.detectAndCompute(img2, kps2, desc2);

  // cv::drawKeypoints(img1, kps1, img1);
  // cv::drawKeypoints(img2, kps2, img2);

  std::vector<int> vMatchIndices;
  int nMatches = matchKps(kps1, kps2, desc1, desc2, vMatchIndices);
  PRINT("nMatches", nMatches, vMatchIndices.size());

  std::vector<cv::Point> vKps1, vKps2;
  for (size_t i = 0; i < vMatchIndices.size(); ++i)
  {
    if (vMatchIndices[i] == -1)
    {
      continue;
    }
    const cv::KeyPoint kp1 = kps1[vMatchIndices[i]];
    const cv::KeyPoint kp2 = kps2[i];
    vKps1.push_back(kp1.pt);
    vKps2.push_back(kp2.pt);
  }

  // std::vector<uchar> inliersH;
  // cv::Mat H = cv::findHomography(vKps1, vKps2, cv::RANSAC, 3.0, inliersH);
  // std::vector<cv::Mat> Rs, ts, Ns;
  // cv::decomposeHomographyMat(H, K, Rs, ts, Ns);

  // for (size_t i = 0; i < Rs.size(); ++i)
  // {
  //   cv::Mat R = Rs[i];
  //   cv::Mat t = ts[i];
  //   cv::Mat Rt;
  //   cv::hconcat(R, t, Rt);
  //   std::cout << "R" << i << ":\n" << R << "\nt" << i << ":\n" << t.t() <<
  //   "\n";
  // }

  for (size_t i = 0; i < vMatchIndices.size(); ++i)
  {
    if (vMatchIndices[i] >= 0)
    {
      cv::KeyPoint kp1 = kps1[vMatchIndices[i]];
      cv::KeyPoint kp2 = kps2[i];
      Misc::draw_line(colorImg, kp1, kp2, cv::Scalar(0, 255, 0));
    }
  }

  cv::imshow("Image2", colorImg);
  // cv::imshow("Image1", img1);
  // cv::imshow("Image2", img2);
  cv::waitKey();

  return 0;
}