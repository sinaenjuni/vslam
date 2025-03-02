#include "camera.h"

#include "misc.h"

Camera::Camera() {}
Camera::Camera(Settings settings)
    : fx(settings.fx),
      fy(settings.fy),
      cx(settings.cx),
      cy(settings.cy),
      bf(settings.bf),
      width(settings.width),
      height(settings.height)
{
  // clang-format off
  K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 
                                  0, fy, cy, 
                                  0, 0, 1);
  // clang-format on
  Kinv = K.inv();
  Kinv /= Kinv.at<double>(2, 2);  // Kinv normalization
}
Camera::Camera(double fx, double fy, double cx, double cy, double bf, int width, int height)
    : fx(fx), fy(fy), cx(cx), cy(cy), bf(bf), width(width), height(height)
{
  K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  Kinv = K.inv();
  Kinv /= Kinv.at<double>(2, 2);  // Kinv normalization
}

Camera::~Camera() {}

void Camera::unproject(const cv::Mat &kps, cv::Mat &kpsn) const
{
  // Coordinates translation from the image coordinates to the normalized camera
  // ones.
  // cv::Mat kps_homo;
  // Matrix::to_homogeneous(kps, kps_homo);

  cv::Mat kps_homo = Matrix::to_homogeneous(kps);
  kps_homo = (Kinv * kps_homo.t()).t();
  kpsn = kps_homo.colRange(0, 2);
}

// void Camera::project(const cv::Mat &kpsn, cv::Mat &kps, cv::Mat &depth) const
// {
//   // Convert from the normalized camera coordinates to the image ones.
//   cv::Mat kpsn_homo;
//   Matrix::to_homogeneous(kpsn, kpsn_homo);
//   kpsn_homo = (K * kpsn_homo.t()).t();

//   kps = kpsn_homo.colRange(0, 2);
//   depth = kpsn_homo.col(2);
//   // The OpenCV not supports the broadcasting.
//   cv::Mat depth_expanded;
//   cv::repeat(depth, 1, 2, depth_expanded);  // Repeat colums twoice
//   kps = kps / depth_expanded;               // Normalized
// }
void Camera::project(const cv::Mat &kpsn, cv::Mat &kps) const
{
  // Convert from the normalized camera coordinates to the image ones.
  // cv::Mat kpsn_homo;
  // Matrix::to_homogeneous(kpsn, kpsn_homo);
  kps = (K * kpsn.t()).t();

  // kps = kps.colRange(0, 2);
  // depth = kpsn.col(2);
  // The OpenCV not supports the broadcasting.
  cv::Mat denominator;
  cv::repeat(kpsn.col(2), 1, 3, denominator);  // Repeat colums twoice
  kps = kps / denominator;                     // Normalized
  kps = kps.colRange(0, 2);
}
