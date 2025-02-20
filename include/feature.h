// #include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

class Frame;

class IFeature_extractor
{
 public:
  virtual ~IFeature_extractor() = default;

  virtual void detect_and_compute(
      cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Mat &desc) = 0;
};

class FAST_ORB_extractor : public IFeature_extractor
{
 private:
  cv::Ptr<cv::ORB> descriptor;
  int n_points;
  int n_levels;
  int width;
  int height;
  std::vector<double_t> &scale2_factors;
  std::vector<double_t> &scale2inv_factors;
  std::vector<cv::Mat> img_pyramid;

 public:
  FAST_ORB_extractor(
      int n_points,
      int n_levels,
      int width,
      int height,
      std::vector<double_t> &scale2_factors,
      std::vector<double_t> &scale2inv_factors);
  ~FAST_ORB_extractor();

  void detect_with_octave(
      cv::Mat &img, std::vector<cv::KeyPoint> &kps, int octave);
  void detect(cv::Mat &img, std::vector<cv::KeyPoint> &kps);
  void detect_and_compute(
      cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Mat &desc) override;
  double_t get_scale2inv(const int octave) const;
  double_t get_scale2(const int octave) const;
};

class IFeature_matcher
{
 public:
  virtual ~IFeature_matcher() = default;

  virtual void feature_matching(
      const Frame *query,
      const Frame *train,
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

  void feature_matching(
      const Frame *query,
      const Frame *train,
      cv::Mat &idx_match_query,
      cv::Mat &idx_match_train) override;
};

namespace Geometry
{
void pose_estimation_with_essential_matrix(
    Frame const *const frame1,
    Frame const *const frame2,
    cv::Mat &idx_match1,
    cv::Mat &idx_match2,
    cv::Mat &R,
    cv::Mat &t);

cv::Mat triangulate(
    Frame const *const frame1,
    Frame const *const frame2,
    const cv::Mat idx_match1,
    const cv::Mat idx_match2);
}  // namespace Geometry