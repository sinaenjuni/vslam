#include "misc.h"

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

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

void Misc::draw_kps(
    const cv::Mat &img,
    const std::vector<cv::KeyPoint> &kps,
    const cv::Scalar color)
{
  for (auto &kp : kps)
  {
    cv::circle(img, kp.pt, 2, color, 1, cv::LINE_AA);
  }
}

void Misc::draw_kps(
    const cv::Mat &img, const cv::Mat &kps, const cv::Scalar color)
{
  for (int i = 0; i < kps.rows; i++)
  {
    cv::circle(
        img,
        static_cast<cv::Point2f>(kps.row(i).colRange(0, 2)),
        2,
        color,
        -1,
        cv::LINE_AA);
  }
}

void Misc::draw_line(
    cv::Mat &img,
    const cv::Mat kps_cur,
    const cv::Mat kps_ref,
    const cv::Scalar color)
{
  for (int i = 0; i < kps_cur.rows; i++)
  {
    cv::line(
        img,
        cv::Point2d(kps_cur.at<double>(i, 0), kps_cur.at<double>(i, 1)),
        cv::Point2d(kps_ref.at<double>(i, 0), kps_ref.at<double>(i, 1)),
        color,
        1,
        cv::LINE_AA);
  }
}

void Misc::draw_line(
    cv::Mat &img,
    const cv::KeyPoint kp1,
    const cv::KeyPoint kp2,
    const cv::Scalar color)
{
  // cv::circle(img, kp1.pt, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
  // cv::circle(img, kp2.pt, 2, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
  cv::line(img, kp1.pt, kp2.pt, color, 1, cv::LINE_AA);
}

std::vector<cv::Mat> Misc::splitToVectorFromCvMat(const cv::Mat &input)
{
  std::vector<cv::Mat> ret;
  ret.reserve(input.rows);
  for (int i = 0; i < input.rows; ++i)
  {
    ret.push_back(input.row(i));
  }
  return ret;
}