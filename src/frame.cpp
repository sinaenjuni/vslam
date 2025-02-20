#include "frame.h"

#include "camera.h"
#include "feature.h"
#include "misc.h"

int get_frame_id()
{
  static int _frame_id = 0;
  return _frame_id++;
}

Frame::Frame() {}
Frame::Frame(Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat &desc)
    : id(get_frame_id())
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

Frame::~Frame() {}

// void Frame::project(
//     const cv::Mat &points4d, cv::Mat &kpsi, cv::Mat &depth) const
// {
//   project_word2camera(points4d, kpsi);
//   project_camera2image(kpsi.colRange(0, 3), kpsi, depth);
//   // cv::Mat camera_coordinate_points = (this->Twc * points4d.t()).t();
//   // camera->project(camera_coordinate_points.colRange(0, 3), kps, depth);
// }

void Frame::project_word2camera(const cv::Mat &points4d, cv::Mat &kpsn) const
{
  kpsn = (this->Twc * points4d.t()).t();
}

// void Frame::project_camera2image(
//     const cv::Mat &points3d, cv::Mat &kpsi, cv::Mat &depth) const
// {
//   camera->project(points3d.colRange(0, 3), kpsi, depth);
// }

// void Frame::project_camera2world(const cv::Mat &kpsn, cv::Mat &kpsw)
// {
//   cv::Mat kpsn_homo;
//   Matrix::to_homogeneous(kpsn, kpsn_homo);
//   cv::Mat Rwc(Twc(cv::Rect(0, 0, 3, 3)));
//   kpsw = (Rwc * kpsn_homo.t()).t();
// }

const cv::Mat Frame::get_kps(const cv::Mat &indices) const
{
  // PRINT(this->kps.rows, this->kps.cols, this->kps.depth(), CV_64F);
  cv::Mat ret = cv::Mat::zeros(indices.rows, this->kps.cols, this->kps.depth());
  // PRINT(ret.rows, ret.cols, ret.depth(), CV_64F);

  for (size_t i = 0; i < indices.rows; i++)
  {
    ret.at<double_t>(i, 0) =
        this->kps.at<double_t>(indices.at<uint16_t>(i, 0), 0);
    ret.at<double_t>(i, 1) =
        this->kps.at<double_t>(indices.at<uint16_t>(i, 0), 1);
  }

  return ret;
}

const cv::Mat Frame::get_kpsn(const cv::Mat &indices) const
{
  cv::Mat ret =
      cv::Mat::zeros(indices.rows, this->kpsn.cols, this->kpsn.depth());
  for (size_t i = 0; i < indices.rows; i++)
  {
    ret.at<double_t>(i, 0) =
        this->kpsn.at<double_t>(indices.at<uint16_t>(i, 0), 0);
    ret.at<double_t>(i, 1) =
        this->kpsn.at<double_t>(indices.at<uint16_t>(i, 0), 1);
  }

  return ret;
}

const cv::Mat Frame::get_octave(const cv::Mat &indices) const
{
  cv::Mat ret =
      cv::Mat::zeros(indices.rows, this->octave.cols, this->octave.depth());
  for (size_t i = 0; i < indices.rows; i++)
  {
    ret.at<uint16_t>(i) = this->kpsn.at<uint16_t>(indices.at<uint16_t>(i));
    ret.at<uint16_t>(i) = this->kpsn.at<uint16_t>(indices.at<uint16_t>(i));
  }
  return ret;
}

// const cv::Mat Frame::get_sigma2inv(const cv::Mat &indices) const
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

// const cv::Mat Frame::get_sigma2(const cv::Mat &indices) const
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

const cv::Mat Frame::get_descriptor(const cv::Mat &indices) const
{
  cv::Mat ret = cv::Mat::zeros(
      indices.rows, this->descriptor.cols, this->descriptor.depth());
  PRINT(ret.size());
  for (size_t i = 0; i < indices.rows; i++)
  {
    this->descriptor.row(i).copyTo(ret.row(i));
    // ret.at<double_t>(i) = this->sigma2.at<double_t>(indices.at<uint16_t>(i,
    // 0));
  }
  return ret;
}
