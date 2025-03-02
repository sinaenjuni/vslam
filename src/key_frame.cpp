#include "key_frame.h"

#include "camera.h"
#include "feature.h"
#include "misc.h"

int get_frame_id()
{
  static int _frame_id = 0;
  return _frame_id++;
}

Key_frame::Key_frame() : id(get_frame_id()) {}

Key_frame::Key_frame(const cv::Mat &img, Camera &camera, FAST_ORB_extractor *feature_extractor)
    : Key_frame()
{
  std::vector<cv::KeyPoint> pts;
  feature_extractor->detect_and_compute(img, pts, this->descriptor);

  int nkps = pts.size();
  this->kps.create(nkps, 2, CV_64F);
  this->octave.create(nkps, 1, CV_16U);

  this->sigma2.create(pts.size(), 1, CV_64F);
  this->sigma2inv.create(pts.size(), 1, CV_64F);

  for (size_t i = 0; i < nkps; i++)
  {
    this->kps.at<double>(i, 0) = pts[i].pt.x;
    this->kps.at<double>(i, 1) = pts[i].pt.y;
    // octave[i] = kps[i].octave;
    this->octave.at<uint8_t>(i) = pts[i].octave;
    sigma2.at<double_t>(i) = feature_extractor->get_scale2(pts[i].octave);
    sigma2inv.at<double_t>(i) = feature_extractor->get_scale2inv(pts[i].octave);
  }

  this->kpsn.create(nkps, 2, CV_64F);
  camera.unproject(this->kps, this->kpsn);
}

Key_frame::Key_frame(Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat &desc) : Key_frame()
{
  kps.create(pts.size(), 2, CV_64F);
  octave.create(pts.size(), 1, CV_16U);
  // sigma2.create(pts.size(), 1, CV_64F);
  // sigma2inv.create(pts.size(), 1, CV_64F);
  descriptor = desc;

  for (size_t i = 0; i < pts.size(); i++)
  {
    kps.at<double>(i, 0) = pts[i].pt.x;
    kps.at<double>(i, 1) = pts[i].pt.y;
    // octave[i] = pts[i].octave;
    octave.at<uint16_t>(i) = pts[i].octave;
    // sigma2.at<double_t>(i) = detector.get_scale2(pts[i].octave);
    // sigma2inv.at<double_t>(i) = detector.get_scale2inv(pts[i].octave);
  }

  kpsn.create(pts.size(), 2, CV_64F);
  camera.unproject(kps, kpsn);
}

Key_frame::~Key_frame() {}

// void Key_frame::project(
//     const cv::Mat &points4d, cv::Mat &kpsi, cv::Mat &depth) const
// {
//   project_word2camera(points4d, kpsi);
//   project_camera2image(kpsi.colRange(0, 3), kpsi, depth);
//   // cv::Mat camera_coordinate_points = (this->Twc * points4d.t()).t();
//   // camera->project(camera_coordinate_points.colRange(0, 3), kps, depth);
// }

// void Key_frame::project_word2camera(const cv::Mat &points4d, cv::Mat &kpsn) const
// {
// kpsn = (this->Twc.to_cvmat() * points4d.t()).t();
// }

// void Key_frame::project_camera2image(
//     const cv::Mat &points3d, cv::Mat &kpsi, cv::Mat &depth) const
// {
//   camera->project(points3d.colRange(0, 3), kpsi, depth);
// }

// void Key_frame::project_camera2world(const cv::Mat &kpsn, cv::Mat &kpsw)
// {
//   cv::Mat kpsn_homo;
//   Matrix::to_homogeneous(kpsn, kpsn_homo);
//   cv::Mat Rwc(Twc(cv::Rect(0, 0, 3, 3)));
//   kpsw = (Rwc * kpsn_homo.t()).t();
// }

// const cv::Mat Key_frame::get_kpsn(const cv::Mat &indices) const
// {
//   cv::Mat ret = cv::Mat::zeros(indices.rows, this->kpsn.cols, this->kpsn.depth());
//   for (size_t i = 0; i < indices.rows; i++)
//   {
//     ret.at<double_t>(i, 0) = this->kpsn.at<double_t>(indices.at<uint16_t>(i, 0), 0);
//     ret.at<double_t>(i, 1) = this->kpsn.at<double_t>(indices.at<uint16_t>(i, 0), 1);
//   }

//   return ret;
// }

// const cv::Mat Key_frame::get_octave(const cv::Mat &indices) const
// {
//   cv::Mat ret = cv::Mat::zeros(indices.rows, this->octave.cols, this->octave.depth());
//   for (size_t i = 0; i < indices.rows; i++)
//   {
//     ret.at<uint16_t>(i) = this->kpsn.at<uint16_t>(indices.at<uint16_t>(i));
//     ret.at<uint16_t>(i) = this->kpsn.at<uint16_t>(indices.at<uint16_t>(i));
//   }
//   return ret;
// }

// const cv::Mat Key_frame::get_sigma2inv(const cv::Mat &indices) const
// {
//   cv::Mat ret = cv::Mat::zeros(
//       indices.rows, this->sigma2inv.cols, this->sigma2inv.depth());
//   for (size_t i = 0; i < indices.rows; i++)
//   {
//     ret.at<double_t>(i) =
//         this->sigma2inv.at<double_t>(indices.at<uint16_t>(i, 0));
//   }

//   return ret;
// }

// const cv::Mat Key_frame::get_sigma2(const cv::Mat &indices) const
// {
//   cv::Mat ret =
//       cv::Mat::zeros(indices.rows, this->sigma2.cols, this->sigma2.depth());
//   for (size_t i = 0; i < indices.rows; i++)
//   {
//     ret.at<double_t>(i) = this->sigma2.at<double_t>(indices.at<uint16_t>(i,
//     0));
//   }

//   return ret;
// }

const cv::Mat Key_frame::get_descriptor(const cv::Mat &indices) const
{
  cv::Mat ret = cv::Mat::zeros(indices.rows, this->descriptor.cols, this->descriptor.depth());
  PRINT(ret.size());
  for (size_t i = 0; i < indices.rows; i++)
  {
    this->descriptor.row(i).copyTo(ret.row(i));
    // ret.at<double_t>(i) = this->sigma2.at<double_t>(indices.at<uint16_t>(i,
    // 0));
  }
  return ret;
}
