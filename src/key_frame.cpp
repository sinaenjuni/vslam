#include "key_frame.h"

#include "camera.h"
#include "feature.h"
#include "misc.h"

int getKeyFrameID()
{
  static int _KEY_FRAME_ID = -1;
  return ++_KEY_FRAME_ID;
}

KeyFrame::KeyFrame() : id(getKeyFrameID()) {}
KeyFrame::KeyFrame(
    cv::Mat &kps,
    cv::Mat &kpsn,
    cv::Mat &descrptor,
    cv::Mat &octave,
    cv::Mat &sigma2,
    cv::Mat &sigma2inv,
    Camera *camera)
    : KeyFrame()
{
  this->kps = kps;
  this->kpsn = kpsn;
  this->descriptor = descrptor;
  this->octave = octave;
  this->sigma2 = sigma2;
  this->sigma2inv = sigma2inv;
  this->camera = camera;
}
KeyFrame::KeyFrame(
    const cv::Mat &img, Camera &camera, FAST_ORB_extractor *feature_extractor)
    : KeyFrame()
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

KeyFrame::KeyFrame(
    Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat &desc)
    : KeyFrame()
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

KeyFrame::~KeyFrame() {}

// void Key_frame::project(
//     const cv::Mat &points4d, cv::Mat &kpsi, cv::Mat &depth) const
// {
//   project_word2camera(points4d, kpsi);
//   project_camera2image(kpsi.colRange(0, 3), kpsi, depth);
//   // cv::Mat camera_coordinate_points = (this->Twc * points4d.t()).t();
//   // camera->project(camera_coordinate_points.colRange(0, 3), kps, depth);
// }

void KeyFrame::projectFrame2World(
    const cv::Mat &kpsFrame, cv::Mat &kpsWorld) const
{
  cv::Mat kpsFrameHomo =
      Matrix::to_homogeneous(Matrix::to_homogeneous(kpsFrame));
  kpsWorld = (this->Twc.to_cvmat() * kpsFrameHomo.t()).t();
  kpsWorld = kpsWorld.colRange(0, 3);
}

void KeyFrame::projectWorld2Frame(
    const cv::Mat &kpsWorld, cv::Mat &kpsFrame) const
{
  kpsFrame = (this->Twc.inverse().to_cvmat() * kpsWorld.t()).t();
  kpsFrame = kpsFrame.colRange(0, 3);
}

void KeyFrame::projectWorld2Frame(
    const cv::Mat &kpsWorld, cv::Mat &kpsFrame, cv::Mat &depth) const
{
  kpsFrame = (this->Twc.inverse().to_cvmat() * kpsWorld.t()).t();
  kpsFrame.col(2).copyTo(depth);
  kpsFrame = kpsFrame.colRange(0, 3);
}

void KeyFrame::projectWorld2Image(
    const cv::Mat &kpsWorld, cv::Mat &kpsImage) const
{
  projectWorld2Frame(kpsWorld, kpsImage);
  camera->project(kpsImage, kpsImage);
}

void KeyFrame::projectWorld2Image(
    const cv::Mat &kpsWorld, cv::Mat &kpsImage, cv::Mat &depth) const
{
  projectWorld2Frame(kpsWorld, kpsImage, depth);
  camera->project(kpsImage, kpsImage);
}

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
//   cv::Mat ret = cv::Mat::zeros(indices.rows, this->kpsn.cols,
//   this->kpsn.depth()); for (size_t i = 0; i < indices.rows; i++)
//   {
//     ret.at<double_t>(i, 0) = this->kpsn.at<double_t>(indices.at<uint16_t>(i,
//     0), 0); ret.at<double_t>(i, 1) =
//     this->kpsn.at<double_t>(indices.at<uint16_t>(i, 0), 1);
//   }

//   return ret;
// }

// const cv::Mat Key_frame::get_octave(const cv::Mat &indices) const
// {
//   cv::Mat ret = cv::Mat::zeros(indices.rows, this->octave.cols,
//   this->octave.depth()); for (size_t i = 0; i < indices.rows; i++)
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

const cv::Mat KeyFrame::get_descriptor(const cv::Mat &indices) const
{
  cv::Mat ret = cv::Mat::zeros(
      indices.rows, this->descriptor.cols, this->descriptor.depth());

  for (size_t i = 0; i < indices.rows; i++)
  {
    this->descriptor.row(i).copyTo(ret.row(i));
    // ret.at<double_t>(i) = this->sigma2.at<double_t>(indices.at<uint16_t>(i,
    // 0));
  }
  return ret;
}
