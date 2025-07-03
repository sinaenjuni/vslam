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
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
  static void computeThreeMaxima(
      std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);
  int matchInGrid(
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
