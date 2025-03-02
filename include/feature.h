#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "settings.h"

class Key_frame;

// class IFeature_extractor
// {
//  public:
//   virtual ~IFeature_extractor() = default;

//   virtual void detect_and_compute(
//       const cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Mat &desc) = 0;
// };
class Camera;

class FAST_ORB_extractor
{
 private:
  cv::Ptr<cv::ORB> descriptor;
  int n_points;
  int n_levels;
  int width;
  int height;
  std::vector<double_t> scale2_factors;
  std::vector<double_t> scale2inv_factors;
  std::vector<cv::Mat> img_pyramid;
  Camera *camera;

 public:
  FAST_ORB_extractor();
  FAST_ORB_extractor(Settings settings, Camera *camera);
  FAST_ORB_extractor(
      int n_points,
      int n_levels,
      int width,
      int height,
      std::vector<double_t> &scale2_factors,
      std::vector<double_t> &scale2inv_factors);
  ~FAST_ORB_extractor();

  void detect(const cv::Mat &img, std::vector<cv::KeyPoint> &kps);
  void detect_with_octave(const cv::Mat &img, std::vector<cv::KeyPoint> &kps, int octave);
  void detect_and_compute(const cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Mat &desc);
  Key_frame *extract(const cv::Mat &img);
  double_t get_scale2inv(const int octave) const;
  double_t get_scale2(const int octave) const;
};

class IFeature_matcher
{
 public:
  virtual ~IFeature_matcher() = default;

  virtual void compute_match(
      const cv::Mat &query,
      const cv::Mat &train,
      cv::Mat &idx_match_query,
      cv::Mat &idx_match_train) = 0;
};

class BFMatcher : public IFeature_matcher
{
 private:
  cv::Ptr<cv::BFMatcher> matcher;

 public:
  BFMatcher(cv::Ptr<cv::BFMatcher> matcher);
  ~BFMatcher();

  void compute_match(
      const cv::Mat &query,
      const cv::Mat &train,
      cv::Mat &idx_match_query,
      cv::Mat &idx_match_train) override;
};
