#pragma once

#include <vector>

#include "frame.h"
#include "settings.h"

// class Camera;

// class IFeatureExtractor
// {
//  public:
//   virtual ~IFeatureExtractor() = default;

//   virtual void detect_and_compute(
//       const cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Mat &desc) = 0;
//   virtual bool extract(const cv::Mat &img, KeyFramePtr keyFrame) = 0;
// };

// class FAST_ORB_extractor : public IFeatureExtractor
class FastOrbExtractor
{
 private:
  int mNPoints;
  int mNLevels;
  int mImgWidth;
  int mImgHeight;
  float mScaleFactor;

  std::vector<int> umax;
  std::vector<float> mvScaleFactors;
  std::vector<float> mvScaleFactorsInv;
  std::vector<int> mvNPointsPerLevels;
  std::vector<cv::Mat> mvImgPyramid;

  int mFastMinThreshold;
  int mFastThreshold;

  std::vector<cv::Point> pattern;

 public:
  void resizeWithLevel(cv::Mat &img, int &level);
  FastOrbExtractor(
      int nPoints,
      int nLevels,
      int imgWidth,
      int imgHeight,
      float scaleFactor,
      int fastMinThreathold = 7,
      int fastThreathold = 20);
  ~FastOrbExtractor();
  void makeImagePyramid(const cv::Mat &img);
  void detectAndCompute(
      const cv::Mat &img,
      std::vector<cv::KeyPoint> &outKeyPoints,
      cv::Mat &outDescriptors);

  std::vector<cv::KeyPoint> detectPerLevel(
      const cv::Mat &img, const int &level, const int &nPoints);
  bool extract(const cv::Mat &img, KeyFrame *keyFrame);
  float get_scale2inv(const int octave) const;
  float get_scale2(const int octave) const;
  std::vector<int> getNPointsPerLevels();
};

inline std::vector<int> FastOrbExtractor::getNPointsPerLevels()
{
  return mvNPointsPerLevels;
}