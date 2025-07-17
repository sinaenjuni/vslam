#include "frame.h"

#include <cstddef>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <utility>
#include <vector>

#include "DBoW2.h"
#include "extractor.h"
#include "map_point.h"
#include "misc.h"

unsigned long getKeyFrameID()
{
  static unsigned long _KEY_FRAME_ID = -1;
  return ++_KEY_FRAME_ID;
}
// bool compareKFsConnections(std::string a, std::string b)
// {
//   int numberA =
//       std::stoi(fs::path(a).filename().replace_extension("").string());
//   int numberB =
//       std::stoi(fs::path(b).filename().replace_extension("").string());
//   return a < b;
//   // if this function returns false, exchage the position of a and b.
// }
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
int KeyFrame::nLevels = 0;
float KeyFrame::scaleFactor = 0.0;
float KeyFrame::minX = 0.0;
float KeyFrame::minY = 0.0;
float KeyFrame::maxX = 0.0;
float KeyFrame::maxY = 0.0;
float KeyFrame::fx = 0.0;
float KeyFrame::fy = 0.0;
float KeyFrame::cx = 0.0;
float KeyFrame::cy = 0.0;
std::vector<float> KeyFrame::scaleFactors = std::vector<float>();
std::vector<float> KeyFrame::invScaleFactors2 = std::vector<float>();
cv::Mat KeyFrame::K = cv::Mat();
cv::Mat KeyFrame::distCoef = cv::Mat();
float KeyFrame::gridColsInv = 0.0;
float KeyFrame::gridRowsInv = 0.0;
float KeyFrame::getInvScaleFactors2(const int level)
{
  return invScaleFactors2[level];
}

KeyFrame::KeyFrame() : mID(getKeyFrameID()), mbFirstConnections(false)
// mTcw(cv::Mat::eye(4, 4, CV_64F)),
// mTwc(cv::Mat::eye(4, 4, CV_64F)),
// mOw(cv::Mat::zeros(3, 1, CV_64F))
{
}

// KeyFrame::KeyFrame(const cv::Mat &img, FastOrbExtractor *featureExtractor)
// {
// featureExtractor->detectAndCompute(img, mvKps, mDescriptors);
// N = mvKps.size();
// PRINT("KeyFrame::KeyFrame", N);
// this->Twc = cv::Mat::eye(4, 4, CV_32F);
// featureExtractor->detectAndCompute(img, mvKps, mDescriptors);

// featureExtractor->detectAndCompute(img, mvKps);
// N = mvKps.size();
// mvMapPoints = std::vector<MapPoint *>(N, nullptr);
// assignFeaturesToGrid();
// }

KeyFrame::KeyFrame(
    const cv::Mat &img,
    cv::Ptr<cv::ORB> featureExtractor,
    OrbVocabulary *pOrbVocabulary)
    : KeyFrame()
{
  featureExtractor->detectAndCompute(img, cv::noArray(), mvKps, mDescriptors);
  this->mN = mvKps.size();
  this->mvMapPoints =
      std::vector<MapPoint *>(this->mN, static_cast<MapPoint *>(nullptr));
  this->mvTrackedkps = std::vector<bool>(this->mN, false);
  this->mTcw = cv::Mat::eye(4, 4, CV_64F);
  this->mTwc = cv::Mat::eye(4, 4, CV_64F);
  this->mOw = cv::Mat::zeros(3, 1, CV_64F);

  setKpsToGrid();
  mpOrbVocabulary = pOrbVocabulary;

  // PRINT("KeyFrame::KeyFrame", mvKps.size());
  // featureExtractor->detectAndCompute(img, mvKps, mDescriptors);

  // featureExtractor->detectAndCompute(img, mvKps);
  // N = mvKps.size();
  // mvMapPoints = std::vector<MapPoint *>(N, nullptr);
  // assignFeaturesToGrid();
}

KeyFrame::KeyFrame(
    const cv::Mat &img,
    FastOrbExtractor *featureExtractor,
    OrbVocabulary *pOrbVocabulary)
    : KeyFrame()
{
  featureExtractor->detectAndCompute(img, mvKps, mDescriptors);
  this->mN = mvKps.size();
  this->mvMapPoints =
      std::vector<MapPoint *>(this->mN, static_cast<MapPoint *>(nullptr));
  this->mvTrackedkps = std::vector<bool>(this->mN, false);
  this->mTcw = cv::Mat::eye(4, 4, CV_64F);
  this->mTwc = cv::Mat::eye(4, 4, CV_64F);
  this->mOw = cv::Mat::zeros(3, 1, CV_64F);

  setKpsToGrid();
  mpOrbVocabulary = pOrbVocabulary;

  // PRINT("KeyFrame::KeyFrame", mvKps.size());
  // featureExtractor->detectAndCompute(img, mvKps, mDescriptors);

  // featureExtractor->detectAndCompute(img, mvKps);
  // N = mvKps.size();
  // mvMapPoints = std::vector<MapPoint *>(N, nullptr);
  // assignFeaturesToGrid();
}

void KeyFrame::initStaticVariables(
    const int &nLevels,
    const float &scaleFactor,
    const int &imgWidth,
    const int &imgHeight,
    const float &fx,
    const float &fy,
    const float &cx,
    const float &cy,
    const float &p1,
    const float &p2,
    const float &k1,
    const float &k2)
{
  KeyFrame::K = cv::Mat::eye(3, 3, CV_64F);
  KeyFrame::K.at<double>(0, 0) = fx;
  KeyFrame::K.at<double>(1, 1) = fy;
  KeyFrame::K.at<double>(0, 2) = cx;
  KeyFrame::K.at<double>(1, 2) = cy;

  KeyFrame::fx = fx;
  KeyFrame::fy = fy;
  KeyFrame::cx = cx;
  KeyFrame::cy = cy;

  KeyFrame::nLevels = nLevels;
  KeyFrame::scaleFactors = std::vector<float>(nLevels, 0);
  KeyFrame::invScaleFactors2 = std::vector<float>(nLevels, 0);
  for (int level = 0; level < nLevels; ++level)
  {
    scaleFactors[level] = std::pow(scaleFactor, level);
    invScaleFactors2[level] = 1 / scaleFactors[level] * scaleFactors[level];
  }

  KeyFrame::distCoef = cv::Mat(4, 1, CV_64F);
  KeyFrame::distCoef.at<double>(0) = p1;
  KeyFrame::distCoef.at<double>(1) = p2;
  KeyFrame::distCoef.at<double>(2) = k1;
  KeyFrame::distCoef.at<double>(3) = k2;

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

  KeyFrame::gridColsInv = static_cast<float>(KeyFrame::GRID_COLS) /
                          (KeyFrame::maxX - KeyFrame::minX);
  KeyFrame::gridRowsInv = static_cast<float>(KeyFrame::GRID_ROWS) /
                          (KeyFrame::maxY - KeyFrame::minY);
}

void KeyFrame::computeBoW()
{
  if (mBowVec.empty() || mFeatVec.empty())
  {
    std::vector<cv::Mat> vDescriptors =
        Misc::splitToVectorFromCvMat(this->mDescriptors);
    mpOrbVocabulary->transform(vDescriptors, mBowVec, mFeatVec, 4);
  }
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
int KeyFrame::getID() const { return this->mID; }
size_t KeyFrame::getN() const { return this->mvKps.size(); }
const std::vector<cv::KeyPoint> &KeyFrame::getKps() const
{
  return this->mvKps;
}
const DBoW2::FeatureVector &KeyFrame::getFeatVec() const
{
  return this->mFeatVec;
}
cv::Mat KeyFrame::getCameraCenter() const { return this->mOw.clone(); }
const cv::Mat &KeyFrame::getDescriptors() const
{
  // cv::Mat ret;
  // mDescriptors.copyTo(ret);
  return this->mDescriptors;
}
void KeyFrame::setPose(const cv::Mat &Tcw)
{
  this->mTcw = Tcw.clone();

  cv::Mat Rwc = Tcw(cv::Rect(0, 0, 3, 3)).t();
  cv::Mat tcw = Tcw(cv::Rect(3, 0, 1, 3));
  mOw = -Rwc * tcw;

  mTwc = cv::Mat::eye(4, 4, CV_64F);
  Rwc.copyTo(mTwc(cv::Rect(0, 0, 3, 3)));
  mOw.copyTo(mTwc(cv::Rect(3, 0, 1, 3)));
}
void KeyFrame::setPose(const cv::Mat &Rcw, const cv::Mat &tcw)
{
  Rcw.copyTo(this->mTcw(cv::Rect(0, 0, 3, 3)));  // x y w h
  tcw.copyTo(this->mTcw(cv::Rect(3, 0, 1, 3)));  // x y w h

  cv::Mat Rwc = Rcw.t();
  mOw = -Rwc * tcw;

  mTwc = cv::Mat::eye(4, 4, CV_64F);
  Rwc.copyTo(mTwc(cv::Rect(0, 0, 3, 3)));
  mOw.copyTo(mTwc(cv::Rect(3, 0, 1, 3)));
}
cv::Mat KeyFrame::getPose() const { return this->mTcw.clone(); }
cv::Mat KeyFrame::getPoseInv() const { return this->mTwc.clone(); }
Eigen::Matrix4d KeyFrame::getPoseEigenInv() const
{
  Eigen::Matrix4d TwcEigen;
  cv::cv2eigen(this->mTwc, TwcEigen);
  return TwcEigen;
}
cv::Mat KeyFrame::getP() const
{
  // PRINT(KeyFrame::K.size(), mTwc(cv::Rect(0, 0, 3, 4)).size());
  // PRINT(KeyFrame::K.type(), mTwc(cv::Rect(0, 0, 3, 4)).type());
  return KeyFrame::K * mTcw(cv::Rect(0, 0, 4, 3));
}
void KeyFrame::addMapPoint(MapPoint *mp, size_t ImgIdx)
{
  this->mvMapPoints[ImgIdx] = mp;
}
std::vector<MapPoint *> KeyFrame::getMapPoints()
{
  // if need mutex here
  return this->mvMapPoints;
}

// void Key_frame::project(
//     const cv::Mat &points4d, cv::Mat &kpsi, cv::Mat &depth) const
// {
//   project_word2camera(points4d, kpsi);
//   project_camera2image(kpsi.colRange(0, 3), kpsi, depth);
//   // cv::Mat camera_coordinate_points = (this->Twc * points4d.t()).t();
//   // camera->project(camera_coordinate_points.colRange(0, 3), kps, depth);
// }

// void KeyFrame::projectFrame2World(
//     const cv::Mat &kpsFrame, cv::Mat &kpsWorld) const
// {
//   cv::Mat kpsFrameHomo =
//       Matrix::to_homogeneous(Matrix::to_homogeneous(kpsFrame));
//   kpsWorld = (this->Twc * kpsFrameHomo.t()).t();
//   kpsWorld = kpsWorld.colRange(0, 3);
// }

// void KeyFrame::projectWorld2Frame(
//     const cv::Mat &kpsWorld, cv::Mat &kpsFrame) const
// {
//   kpsFrame = (this->Twc.inv() * kpsWorld.t()).t();
//   kpsFrame = kpsFrame.colRange(0, 3);
// }

// void KeyFrame::projectWorld2Frame(
//     const cv::Mat &kpsWorld, cv::Mat &kpsFrame, cv::Mat &depth) const
// {
//   kpsFrame = (this->Twc.inv() * kpsWorld.t()).t();
//   kpsFrame.col(2).copyTo(depth);
//   kpsFrame = kpsFrame.colRange(0, 3);
// }

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

  for (int i = 0; i < indices.rows; i++)
  {
    this->mDescriptors.row(i).copyTo(ret.row(i));
    // ret.at<double_t>(i) = this->sigma2.at<double_t>(indices.at<uint16_t>(i,
    // 0));
  }
  return ret;
}

void KeyFrame::setKpsToGrid()
{
  int nReserve = 0.5f * mN / (GRID_COLS * GRID_ROWS);
  for (size_t row = 0; row < GRID_ROWS; row++)
  {
    for (size_t col = 0; col < GRID_COLS; col++)
    {
      mvGrid[row][col].reserve(nReserve);
    }
  }

  for (int i = 0; i < mN; i++)
  {
    const cv::KeyPoint &kp = mvKps[i];
    int posX, posY;
    if (getPosInGrid(kp, posX, posY))
    {
      mvGrid[posY][posX].push_back(i);
    }
  }
}

bool KeyFrame::getPosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
  posX = cvRound((kp.pt.x - KeyFrame::minX) * KeyFrame::gridColsInv);
  posY = cvRound((kp.pt.y - KeyFrame::minY) * KeyFrame::gridRowsInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the
  // image
  if (posX < 0 || posX >= KeyFrame::GRID_COLS ||  //
      posY < 0 || posY >= KeyFrame::GRID_ROWS)
    return false;
  return true;
}

std::vector<size_t> KeyFrame::getKpsInGrid(
    const float &x,
    const float &y,
    const float &r,
    const int minLevel,
    const int maxLevel) const
{
  std::vector<size_t> vIndices;
  vIndices.reserve(mN);

  const float gridWidthInv = KeyFrame::gridColsInv;
  const float gridHeightInv = KeyFrame::gridRowsInv;
  const int nMinCellX =
      std::max(0, cvRound((x - KeyFrame::minX - r) * gridWidthInv));
  const int nMaxCellX = std::min(
      static_cast<int>(GRID_COLS) - 1,
      cvRound((x - KeyFrame::minX + r) * gridWidthInv));
  const int nMinCellY =
      std::max(0, cvRound((y - KeyFrame::minY - r) * gridHeightInv));
  const int nMaxCellY = std::min(
      static_cast<int>(GRID_ROWS) - 1,
      cvRound((y - KeyFrame::minY + r) * gridHeightInv));

  if (nMaxCellX < 0 || nMinCellX >= GRID_COLS ||  //
      nMaxCellY < 0 || nMinCellY >= GRID_ROWS)
  {
    std::cerr << "Warning: Search area out of grid bounds (x=" << x
              << ", y=" << y << ", r=" << r << ")" << std::endl;
    return vIndices;
  }
  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (int iy = nMinCellY; iy <= nMaxCellY; ++iy)  // row frist iteration
  {
    for (int ix = nMinCellX; ix <= nMaxCellX; ++ix)  // column iteration
    {
      const std::vector<size_t> &vKpsInCell = mvGrid[iy][ix];
      // mGrid[posY][posX]
      if (vKpsInCell.empty()) continue;

      for (const size_t &kpsInCell : vKpsInCell)
      {
        const cv::KeyPoint &kp = mvKps[kpsInCell];
        if (bCheckLevels)
        {
          if (kp.octave < minLevel || (maxLevel >= 0 && kp.octave > maxLevel))
          {
            continue;
          }
        }

        const float distx = kp.pt.x - x;
        const float disty = kp.pt.y - y;
        // const float scale = 1.0f / (1 << kpUn.octave);
        // if (distx * distx + disty * disty < r * r * scale * scale)
        // {
        //   vIndices.push_back(kpsInCell);
        // }
        if (fabs(distx) < r && fabs(disty) < r)
        {
          vIndices.push_back(kpsInCell);
        }
      }
    }
  }

  return vIndices;
}
void KeyFrame::addConnection(KeyFrame *pKF, int weight)
{
  {
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF] = weight;
    else if (mConnectedKeyFrameWeights[pKF] != weight)
      mConnectedKeyFrameWeights[pKF] = weight;
    else
      return;
  }

  updateBestCovisibles();
}
void KeyFrame::updateBestCovisibles()
{
  std::unique_lock<std::mutex> lock(mMutexConnections);
  std::vector<std::pair<int, KeyFrame *>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (std::map<KeyFrame *, int>::iterator
           mit = mConnectedKeyFrameWeights.begin(),
           mend = mConnectedKeyFrameWeights.end();
       mit != mend;
       mit++)
    vPairs.push_back(std::make_pair(mit->second, mit->first));

  sort(vPairs.begin(), vPairs.end());
  std::list<KeyFrame *> lKFs;
  std::list<int> lWs;
  for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
  {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  mvpOrderedConnectedKeyFrames =
      std::vector<KeyFrame *>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}
void KeyFrame::addChild(KeyFrame *pKF)
{
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mspChildrens.insert(pKF);
}
void KeyFrame::updateConnections()
{
  std::map<KeyFrame *, int> KFsCounter;
  std::vector<MapPoint *> vMPs;

  {
    // if need a mutex here
    vMPs = mvMapPoints;  // 현재 프레임에서 보이는 MP
  }

  for (std::vector<MapPoint *>::iterator iterMP = vMPs.begin();
       iterMP != vMPs.end();
       ++iterMP)
  {
    MapPoint *pMP = *iterMP;
    if (!pMP)
    {
      continue;
    }
    std::map<KeyFrame *, uvIdx> observations = pMP->getObservations();
    // 현재 프레임에서 보이는 MP를 공유하는 KFs를 가져옴

    for (std::map<KeyFrame *, uvIdx>::iterator iterObs = observations.begin();
         iterObs != observations.end();
         ++iterObs)
    {
      if (iterObs->first == this)
      {
        continue;  // skip self
      }
      ++KFsCounter[iterObs->first];
    }
  }

  // This should not happen
  if (KFsCounter.empty())
  {
    return;
  }

  int nMax = 0;
  KeyFrame *pKFmax = static_cast<KeyFrame *>(nullptr);
  std::vector<std::pair<int, KeyFrame *>> vPairs;
  int th = 15;  // covisibility threshold
  for (auto [pKF, nCount] : KFsCounter)
  {
    if (nCount < nMax)
    {
      nMax = nCount;
      pKFmax = pKF;
    }
    if (nCount >= th)
    {
      vPairs.push_back(std::make_pair(nCount, pKF));
      pKF->addConnection(this, nCount);
    }
  }

  if (vPairs.empty())  // 만약 th보다 많은 MP를 공유하는 KF가 없다면, 가장 많은
                       // MP를 공유하는 KF와 연결한다.
  {
    vPairs.push_back(std::make_pair(nMax, pKFmax));
    pKFmax->addConnection(this, nMax);  // 가장 많은
  }

  sort(vPairs.begin(), vPairs.end());  // 오름차순 정렬 후
  std::list<KeyFrame *> lKFs;
  std::list<int> lWs;
  for (size_t i = 0; i < vPairs.size(); i++)
  {
    lKFs.push_front(vPairs[i].second);  // 앞에서부터 저장 (내림차순 정렬)
    lWs.push_front(vPairs[i].first);
  }

  {
    std::unique_lock<std::mutex> lockCon(mMutexConnections);

    // mspConnectedKeyFrames = spConnectedKeyFrames;
    mConnectedKeyFrameWeights = KFsCounter;
    mvpOrderedConnectedKeyFrames =
        std::vector<KeyFrame *>(lKFs.begin(), lKFs.end());
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

    if (mbFirstConnections && mID != 0)
    {
      mpParentKF = mvpOrderedConnectedKeyFrames.front();
      // 가장 많이 연결된 KF를 저장
      mpParentKF->addChild(this);
      // 가장 많은 MP를 공유하는 KF에 현재 KF를 자식으로 추가한다.
      mbFirstConnections = false;  // 이 과정은 KF생성 후 최초 한 번만 수행된다.
    }
  }
}