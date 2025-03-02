#include "geometry.h"

#include "key_frame.h"
#include "misc.h"

namespace Geometry
{
// Calculate T(transformation matrix) from a points2 to a points1
// resultly, can get a T12 matrix
void pose_estimation_with_essential_matrix(
    const cv::Mat &kpsn1,
    const cv::Mat &kpsn2,
    cv::Mat &idx_match1,
    cv::Mat &idx_match2,
    cv::Mat &R,
    cv::Mat &t)
{
  // cv::Mat points1(frame1->get_kpsn(idx_match1));
  // cv::Mat points2(frame2->get_kpsn(idx_match2));
  cv::Mat points1 = Matrix::slice_cvmat(kpsn1, idx_match1);
  cv::Mat points2 = Matrix::slice_cvmat(kpsn2, idx_match2);

  cv::Mat E, inlier;
  // Calculate a transformation matrix from the points1 to the points`2.
  E = cv::findEssentialMat(
      points1, points2, 1.0, cv::Point2d(0, 0),
      // cv::RANSAC, 0.999, 0.0004, inlier);
      cv::RANSAC, 0.999, 0.004, inlier);
  // PRINT(E);

  // cv::Mat R, t;
  cv::recoverPose(E, points1, points2, R, t, 1.0, cv::Point2d(0, 0), inlier);
  // PRINT(R);
  // PRINT(t);

  cv::Mat inlier1(0, 0, CV_16U), inlier2(0, 0, CV_16U);
  for (size_t i = 0; i < inlier.rows; i++)
  {
    if (inlier.at<uchar>(i) == 1)
    {
      inlier1.push_back(idx_match1.row(i));
      inlier2.push_back(idx_match2.row(i));
    }
  }

  inlier1.copyTo(idx_match1);
  inlier2.copyTo(idx_match2);
}

cv::Mat triangulate(
    Key_frame const *const frame1,
    Key_frame const *const frame2,
    const cv::Mat idx_match1,
    const cv::Mat idx_match2)
{
  cv::Mat points4d;
  cv::triangulatePoints(
      frame1->get_Twc().inverse()(4, 3).to_cvmat(), frame2->get_Twc().inverse()(4, 3).to_cvmat(),
      frame1->get_kpsn(idx_match1).t(), frame2->get_kpsn(idx_match2).t(), points4d);
  points4d = points4d.t();
  points4d = points4d / cv::repeat(points4d.col(3), 1, 4);

  // return points4d.colRange(0, 3);
  return points4d;
}
}  // namespace Geometry
