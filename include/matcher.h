#include <opencv2/features2d.hpp>

#include "frame.h"

class Matcher
{
 private:
  cv::Ptr<cv::BFMatcher> matcher;
  bool mbCheckOrientation;
  float mfRatioTest;

 public:
  Matcher(float ratioTest = 0.6f, bool checkOrientation = true);
  ~Matcher();
  int matching(
      KeyFrame *F1,
      KeyFrame *F2,
      std::vector<int> &matchF1,
      std::vector<int> &matchF2);
  int knnMatch(
      KeyFrame *F1,
      KeyFrame *F2,
      std::vector<int> &matchF1,
      std::vector<int> &matchF2);
  int matchingForInitialization(
      KeyFrame &F1,
      KeyFrame &F2,
      cv::Mat &idx_match_query,
      cv::Mat &idx_match_train);
};
