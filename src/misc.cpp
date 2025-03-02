#include "misc.h"

namespace Matrix
{
// void to_homogeneous(const cv::Mat &input, cv::Mat &output)
// {
//   output.create(input.rows, 3, CV_64F);
//   // The homogeneous coordinates shape are (N, 3)
//   output.col(2).setTo(1.0);
//   // A last element in a homogeneous coordinate is 1
//   input.copyTo(output.colRange(0, input.cols));
// }

cv::Mat to_homogeneous(const cv::Mat &input)
{
  cv::Mat ret(input.rows, input.cols + 1, CV_64F);
  // output.create(input.rows, 3, CV_64F);
  // The homogeneous coordinates shape are (N, 3)
  ret.col(input.cols).setTo(1.0);
  // A last element in a homogeneous coordinate is 1
  input.copyTo(ret.colRange(0, input.cols));
  return ret;
}

cv::Mat slice_cvmat(const cv::Mat &input, const cv::Mat &indices)
{
  cv::Mat ret = cv::Mat::zeros(indices.rows, input.cols, input.depth());
  indices.forEach<uint16_t>(
      [&ret, &input](const uint16_t &value, const int *position)
      {
        const uchar *src = input.ptr(value);
        uchar *dst = ret.ptr(position[0]);
        memcpy(dst, src, input.cols * input.elemSize());
        // input.row(value).copyTo(ret.row(position[0]));
      });
  return ret;
}
}  // namespace Matrix

namespace Misc
{
void draw_kps(const cv::Mat &img, const std::vector<cv::KeyPoint> &kps, const cv::Scalar color)
{
  for (auto &kp : kps)
  {
    cv::circle(img, kp.pt, 2, color, -1, cv::LINE_AA);
  }
}

void draw_kps(const cv::Mat &img, const cv::Mat &kps, const cv::Scalar color)
{
  for (size_t i = 0; i < kps.rows; i++)
  {
    cv::circle(img, static_cast<cv::Point2f>(kps.row(i)), 2, color, -1, cv::LINE_AA);
  }
}

void draw_line(cv::Mat &img, const cv::Mat kps_cur, const cv::Mat kps_ref, cv::Scalar color)
{
  for (size_t i = 0; i < kps_cur.rows; i++)
  {
    cv::line(
        img, cv::Point2d(kps_cur.at<double>(i, 0), kps_cur.at<double>(i, 1)),
        cv::Point2d(kps_ref.at<double>(i, 0), kps_ref.at<double>(i, 1)), color, 1, cv::LINE_AA);
  }
}

}  // namespace Misc
