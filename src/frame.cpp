#include "frame.h"

#include <opencv2/core/hal/interface.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "feature.h"
#include "misc.h"

int getKeyFrameID()
{
  static int _KEY_FRAME_ID = -1;
  return ++_KEY_FRAME_ID;
}

void computeUndistoredImageBounds(
    const int &imgWidth,
    const int &imgHeight,
    const cv::Mat &distCoef,
    const cv::Mat &K,
    float &minX,
    float &minY,
    float &maxX,
    float &maxY)
{
  if (distCoef.at<float>(0) != 0.0)
  {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;

    mat.at<float>(1, 0) = imgWidth;
    mat.at<float>(1, 1) = 0.0;

    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imgHeight;

    mat.at<float>(3, 0) = imgWidth;
    mat.at<float>(3, 1) = imgHeight;
    /*
   (0, 0)        (640, 0)
    +--------------+
    |              |
    |              |
    +--------------+
    (0, 480)    (640, 480)
    */

    // Undistort corners
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, K, distCoef, cv::Mat(), K);
    mat = mat.reshape(1);

    /*
    (-16, -16)    (656, -16)
        +------------+
        |            |
        |            |
        +------------+
    (-16, 496)  (656, 496)
    */

    minX = std::min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    minY = std::min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    maxX = std::max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    maxY = std::max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  }
  else
  {
    minX = 0.0f;
    minY = 0.0f;
    maxX = imgWidth;
    maxY = imgHeight;
  }
  return;
}

float KeyFrame::minX = 0.0;
float KeyFrame::minY = 0.0;
float KeyFrame::maxX = 0.0;
float KeyFrame::maxY = 0.0;
float KeyFrame::fx = 0.0;
float KeyFrame::fy = 0.0;
float KeyFrame::cx = 0.0;
float KeyFrame::cy = 0.0;
cv::Mat KeyFrame::K = cv::Mat();
cv::Mat KeyFrame::distCoef = cv::Mat();
float KeyFrame::gridColsInv = 0.0;
float KeyFrame::gridRowsInv = 0.0;

KeyFrame::KeyFrame() : id(getKeyFrameID()), Twc(cv::Mat::eye(4, 4, CV_64F)) {}
KeyFrame::KeyFrame(const cv::Mat &img, FastOrbExtractor *featureExtractor)
{
  featureExtractor->detect_and_compute(img, mvKps, mDescriptors);
  N = mvKps.size();
  mvMapPoints = std::vector<MapPoint *>(N, nullptr);
  assignFeaturesToGrid();
}
KeyFrame::KeyFrame(
    cv::Mat &kps,
    cv::Mat &kpsn,
    cv::Mat &descrptor,
    cv::Mat &octave,
    cv::Mat &sigma2,
    cv::Mat &sigma2inv)
    : KeyFrame()
{
  this->kps = kps;
  this->kpsn = kpsn;
  this->mDescriptors = descrptor;
  this->octave = octave;
  this->sigma2 = sigma2;
  this->sigma2inv = sigma2inv;
}

void KeyFrame::initStaticVariables(
    const int &imgWidth,
    const int &imgHeight,
    const float fx,
    const float fy,
    const float cx,
    const float cy,
    const float &p1,
    const float &p2,
    const float &k1,
    const float &k2)
{
  KeyFrame::K = cv::Mat::eye(3, 3, CV_32F);
  KeyFrame::K.at<float>(0, 0) = fx;
  KeyFrame::K.at<float>(1, 1) = fy;
  KeyFrame::K.at<float>(0, 2) = cx;
  KeyFrame::K.at<float>(1, 2) = cy;

  KeyFrame::fx = fx;
  KeyFrame::fy = fy;
  KeyFrame::cx = cx;
  KeyFrame::cy = cy;

  KeyFrame::distCoef = cv::Mat(4, 1, CV_32F);
  KeyFrame::distCoef.at<float>(0) = p1;
  KeyFrame::distCoef.at<float>(1) = p2;
  KeyFrame::distCoef.at<float>(2) = k1;
  KeyFrame::distCoef.at<float>(3) = k2;

  // cv::Mat matl(4, 2, CV_32F);
  // if (KeyFrame::distCoef.at<float>(0) != 0.0)
  if (p1 != 0.0f || p2 != 0.0f || k1 != 0.0f || k2 != 0.0f)
  {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;

    mat.at<float>(1, 0) = imgWidth;
    mat.at<float>(1, 1) = 0.0;

    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imgHeight;

    mat.at<float>(3, 0) = imgWidth;
    mat.at<float>(3, 1) = imgHeight;
    /*
   (0, 0)        (640, 0)
    +--------------+
    |              |
    |              |
    +--------------+
    (0, 480)    (640, 480)
    */

    // Undistort corners
    mat = mat.reshape(2);
    cv::undistortPoints(
        mat, mat, KeyFrame::K, KeyFrame::distCoef, cv::Mat(), KeyFrame::K);
    mat = mat.reshape(1);

    /*
    (-16, -16)    (656, -16)
        +------------+
        |            |
        |            |
        +------------+
    (-16, 496)  (656, 496)
    */

    KeyFrame::minX = std::min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    KeyFrame::maxX = std::max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    KeyFrame::minY = std::min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    KeyFrame::maxY = std::max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  }
  else
  {
    KeyFrame::minX = 0.0f;
    KeyFrame::minY = 0.0f;
    KeyFrame::maxX = imgWidth;
    KeyFrame::maxY = imgHeight;
  }

  KeyFrame::gridColsInv =
      KeyFrame::GRID_COLS / (KeyFrame::maxX - KeyFrame::minX);
  KeyFrame::gridRowsInv =
      KeyFrame::GRID_ROWS / (KeyFrame::maxY - KeyFrame::minY);
}

// KeyFrame::KeyFrame(const cv::Mat &img, Camera &camera, FAST_ORB_extractor
// *feature_extractor)
//     : KeyFrame()
// {
//   std::vector<cv::KeyPoint> pts;
//   feature_extractor->detect_and_compute(img, pts, this->descriptor);

//   int nkps = pts.size();
//   this->kps.create(nkps, 2, CV_64F);
//   this->octave.create(nkps, 1, CV_16U);

//   this->sigma2.create(pts.size(), 1, CV_64F);
//   this->sigma2inv.create(pts.size(), 1, CV_64F);

//   for (size_t i = 0; i < nkps; i++)
//   {
//     this->kps.at<double>(i, 0) = pts[i].pt.x;
//     this->kps.at<double>(i, 1) = pts[i].pt.y;
//     // octave[i] = kps[i].octave;
//     this->octave.at<uint8_t>(i) = pts[i].octave;
//     sigma2.at<double_t>(i) = feature_extractor->get_scale2(pts[i].octave);
//     sigma2inv.at<double_t>(i) =
//     feature_extractor->get_scale2inv(pts[i].octave);
//   }

//   this->kpsn.create(nkps, 2, CV_64F);
//   camera.unproject(this->kps, this->kpsn);
// }

// KeyFrame::KeyFrame(Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat
// &desc) : KeyFrame()
// {
//   kps.create(pts.size(), 2, CV_64F);
//   octave.create(pts.size(), 1, CV_16U);
//   // sigma2.create(pts.size(), 1, CV_64F);
//   // sigma2inv.create(pts.size(), 1, CV_64F);
//   descriptor = desc;

//   for (size_t i = 0; i < pts.size(); i++)
//   {
//     kps.at<double>(i, 0) = pts[i].pt.x;
//     kps.at<double>(i, 1) = pts[i].pt.y;
//     // octave[i] = pts[i].octave;
//     octave.at<uint16_t>(i) = pts[i].octave;
//     // sigma2.at<double_t>(i) = detector.get_scale2(pts[i].octave);
//     // sigma2inv.at<double_t>(i) = detector.get_scale2inv(pts[i].octave);
//   }

//   kpsn.create(pts.size(), 2, CV_64F);
//   camera.unproject(kps, kpsn);
// }

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
  kpsWorld = (this->Twc * kpsFrameHomo.t()).t();
  kpsWorld = kpsWorld.colRange(0, 3);
}

void KeyFrame::projectWorld2Frame(
    const cv::Mat &kpsWorld, cv::Mat &kpsFrame) const
{
  kpsFrame = (this->Twc.inv() * kpsWorld.t()).t();
  kpsFrame = kpsFrame.colRange(0, 3);
}

void KeyFrame::projectWorld2Frame(
    const cv::Mat &kpsWorld, cv::Mat &kpsFrame, cv::Mat &depth) const
{
  kpsFrame = (this->Twc.inv() * kpsWorld.t()).t();
  kpsFrame.col(2).copyTo(depth);
  kpsFrame = kpsFrame.colRange(0, 3);
}

// void KeyFrame::projectWorld2Image(const cv::Mat &kpsWorld, cv::Mat &kpsImage)
// const
// {
//   projectWorld2Frame(kpsWorld, kpsImage);
//   camera->project(kpsImage, kpsImage);
// }

// void KeyFrame::projectWorld2Image(const cv::Mat &kpsWorld, cv::Mat &kpsImage,
// cv::Mat &depth) const
// {
//   projectWorld2Frame(kpsWorld, kpsImage, depth);
//   camera->project(kpsImage, kpsImage);
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
      indices.rows, this->mDescriptors.cols, this->mDescriptors.depth());

  for (size_t i = 0; i < indices.rows; i++)
  {
    this->mDescriptors.row(i).copyTo(ret.row(i));
    // ret.at<double_t>(i) = this->sigma2.at<double_t>(indices.at<uint16_t>(i,
    // 0));
  }
  return ret;
}

void KeyFrame::assignFeaturesToGrid()
{
  int nReserve = 0.5f * N / (GRID_COLS * GRID_ROWS);
  for (unsigned int i = 0; i < GRID_COLS; i++)
  {
    for (unsigned int j = 0; j < GRID_ROWS; j++)
    {
      mvGrid[i][j].reserve(nReserve);
    }
  }

  for (int i = 0; i < N; i++)
  {
    const cv::KeyPoint &kp = mvKps[i];

    int posX, posY;
    if (calcPosInGrid(kp, posX, posY))
    {
      mvGrid[posX][posY].push_back(i);
    }
  }
}

bool KeyFrame::calcPosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
  posX = round((kp.pt.x - KeyFrame::minX) * KeyFrame::gridColsInv);
  posY = round((kp.pt.y - KeyFrame::minY) * KeyFrame::gridRowsInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the
  // image
  if (posX < 0 || posX >= KeyFrame::GRID_COLS || posY < 0 ||
      posY >= KeyFrame::GRID_ROWS)
    return false;

  return true;
}