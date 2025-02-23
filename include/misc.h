#pragma once

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>

// namespace fs = std::__fs::filesystem;

#define PRINT1(a) std::cout << a << '\n'
#define PRINT2(a, b) std::cout << a << ", " << b << '\n'
#define PRINT3(a, b, c) std::cout << a << ", " << b << ", " << c << '\n'
#define PRINT4(a, b, c, d) std::cout << a << ", " << b << ", " << c << ", " << d << '\n'
#define PRINT5(a, b, c, d, e) \
  std::cout << a << ", " << b << ", " << c << ", " << d << ", " << e << '\n'

#define VA_GENERIC(_1, _2, _3, _4, _5, x, ...) x
#define PRINT(...) VA_GENERIC(__VA_ARGS__, PRINT5, PRINT4, PRINT3, PRINT2, PRINT1)(__VA_ARGS__)

#define SHAPE(x) std::cout << x.rows() << " X " << x.cols() << '\n'
#define CVSH(x) std::cout << x.rows << " X " << x.cols << " X " << x.channels() << '\n'
#define DTYPE double_t
#define CVTYPE(x) PRINT(cv::typeToString(x.type()))

// int get_num(const std::string &s);
// bool compare(const fs::path a, const fs::path b);
// bool check_file_extensions(const fs::path &file);

namespace Misc
{
void draw_kps(
    const cv::Mat &img,
    const std::vector<cv::KeyPoint> &kps,
    cv::Scalar color = cv::Scalar(255, 0, 0));

void draw_kps(const cv::Mat &img, const cv::Mat &kps, cv::Scalar color = cv::Scalar(255, 0, 0));

void draw_line(
    cv::Mat &img,
    const cv::Mat kps_cur,
    const cv::Mat kps_ref,
    cv::Scalar color = cv::Scalar(0, 255, 0));

cv::Mat slice_matrix(const cv::Mat &matrix, const cv::Mat &indices);

}  // namespace Misc

namespace Matrix
{
void to_homogeneous(const cv::Mat &input, cv::Mat &output);
cv::Mat slice_cvmat(const cv::Mat &input, const cv::Mat &indices);
}  // namespace Matrix