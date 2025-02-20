#include "misc.h"

namespace Matrix
{
void to_homogeneous(const cv::Mat &input, cv::Mat &output)
{
  output.create(input.rows, 3, CV_64F);
  // The homogeneous coordinates shape are (N, 3)
  output.col(2).setTo(1.0);
  // A last element in a homogeneous coordinate is 1
  input.copyTo(output.colRange(0, input.cols));
}
}  // namespace Matrix

namespace Misc
{
void draw_kps(cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Scalar color)
{
  for (auto &kp : kps)
  {
    cv::circle(img, kp.pt, 2, color, -1, cv::LINE_AA);
  }
}

void draw_kps(cv::Mat &img, cv::Mat &kps, cv::Scalar color)
{
  for (size_t i = 0; i < kps.rows; i++)
  {
    cv::circle(
        img, static_cast<cv::Point2f>(kps.row(i)), 2, color, -1, cv::LINE_AA);
  }
}

void draw_line(
    cv::Mat &img,
    const cv::Mat kps_cur,
    const cv::Mat kps_ref,
    cv::Scalar color)
{
  for (size_t i = 0; i < kps_cur.rows; i++)
  {
    cv::line(
        img, cv::Point2d(kps_cur.at<double>(i, 0), kps_cur.at<double>(i, 1)),
        cv::Point2d(kps_ref.at<double>(i, 0), kps_ref.at<double>(i, 1)), color,
        1, cv::LINE_AA);
  }
}
cv::Mat slice_matrix(const cv::Mat &matrix, const cv::Mat &indices)
{
  // CV_16F
  // cv::Mat ret = cv::Mat::zeros(indices.rows, matrix.cols, matrix.type());
  // indices.forEach(
  //     [&ret]()->void{

  //     }
  // );
  return cv::Mat();
}
}  // namespace Misc
