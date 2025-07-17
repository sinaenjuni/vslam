#pragma once

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <vector>

// namespace fs = std::__fs::filesystem;

#define PRINT1(a) std::cout << a << '\n'
#define PRINT2(a, b) std::cout << a << ", " << b << '\n'
#define PRINT3(a, b, c) std::cout << a << ", " << b << ", " << c << '\n'
#define PRINT4(a, b, c, d) \
  std::cout << a << ", " << b << ", " << c << ", " << d << '\n'
#define PRINT5(a, b, c, d, e) \
  std::cout << a << ", " << b << ", " << c << ", " << d << ", " << e << '\n'
#define PRINT6(a, b, c, d, e, f)                                             \
  std::cout << a << ", " << b << ", " << c << ", " << d << ", " << e << ", " \
            << f << '\n'

#define VA_GENERIC(_1, _2, _3, _4, _5, _6, x, ...) x
#define PRINT(...)                                                         \
  VA_GENERIC(__VA_ARGS__, PRINT6, PRINT5, PRINT4, PRINT3, PRINT2, PRINT1)( \
      __VA_ARGS__)

#define SHAPE(x) std::cout << x.rows() << " X " << x.cols() << '\n'
#define CVSH(x) \
  std::cout << x.rows << " X " << x.cols << " X " << x.channels() << '\n'
#define DTYPE double_t
#define CVTYPE(x) PRINT(cv::typeToString(x.type()))

// int get_num(const std::string &s);
// bool compare(const fs::path a, const fs::path b);
// bool check_file_extensions(const fs::path &file);

class Misc
{
 public:
  static void draw_kps(
      const cv::Mat &img,
      const std::vector<cv::KeyPoint> &kps,
      cv::Scalar color = cv::Scalar(255, 0, 0));

  static void draw_kps(
      const cv::Mat &img,
      const cv::Mat &kps,
      cv::Scalar color = cv::Scalar(255, 0, 0));

  static void draw_line(
      cv::Mat &img,
      const cv::Mat kps_cur,
      const cv::Mat kps_ref,
      const cv::Scalar color = cv::Scalar(0, 255, 0));
  static void draw_line(
      cv::Mat &img,
      const cv::KeyPoint kp1,
      const cv::KeyPoint kp2,
      const cv::Scalar color = cv::Scalar(0, 255, 0));

  static std::vector<cv::Mat> splitToVectorFromCvMat(const cv::Mat &input);
};

namespace Matrix
{
// void to_homogeneous(const cv::Mat &input, cv::Mat &output);
cv::Mat to_homogeneous(const cv::Mat &input);
cv::Mat slice_cvmat(const cv::Mat &input, const cv::Mat &indices);
}  // namespace Matrix