#pragma once

#include <cstddef>
#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <unordered_map>
#include <vector>

#include "entities.h"
#include "map_point.h"
#include "misc.h"

class FastOrbExtractor;
class KeyFrame
{
 public:
  // Undistorted Image bounds
  static float minX, minY, maxX, maxY;
  static float fx, fy, cx, cy;
  static cv::Mat K;
  static cv::Mat distCoef;

  static constexpr size_t GRID_COLS = 64;
  static constexpr size_t GRID_ROWS = 48;
  static float gridColsInv;
  static float gridRowsInv;

 private:
  int id = -1;
  int N = -1;

  std::vector<size_t> mvGrid[GRID_ROWS][GRID_COLS];

  // KeyPoints, normalized KeyPoints and, descriptors
  std::vector<cv::KeyPoint> mvKps;
  std::vector<MapPoint *> mvMapPoints;
  cv::Mat mDescriptors;

  cv::Mat kps;
  cv::Mat kpsn;
  cv::Mat octave;
  cv::Mat idx_tracked_points;
  // Tmat Twc;
  cv::Mat Twc;

  cv::Mat sigma2;
  cv::Mat sigma2inv;

  std::unordered_map<MapPointID, ImgPointIdx> observedPoints;

  void setKpsToGrid();
  bool getPosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

 public:
  KeyFrame();
  KeyFrame(const cv::Mat &img, FastOrbExtractor *featureExtractor);
  KeyFrame(const cv::Mat &img, cv::Ptr<cv::ORB> featureExtractor);
  KeyFrame(
      cv::Mat &kps,
      cv::Mat &kpsn,
      cv::Mat &mDescriptors,
      cv::Mat &octave,
      cv::Mat &sigma2,
      cv::Mat &sigma2inv);
  ~KeyFrame();

  static void initStaticVariables(
      const int &imgWidth,
      const int &imgHeight,
      const float fx,
      const float fy,
      const float cx,
      const float cy,
      const float &p1,
      const float &p2,
      const float &k1,
      const float &k2);

  inline const int getID() const { return this->id; };
  inline const int getN() const { return this->N; }
  std::vector<size_t> getKpsInGrid(
      const float &x,
      const float &y,
      const float &r,
      const int minLevel,
      const int maxLevel) const;

  inline const int get_nkps() const { return this->kps.rows; };

  // inline const cv::Mat &getKps() const { return this->kps; };
  inline const std::vector<cv::KeyPoint> &getKps() const
  {
    return this->mvKps;
  };
  inline const cv::Mat getKps(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->kps, indices);
  };

  inline const cv::Mat &get_kpsn() const { return this->kpsn; };
  inline const cv::Mat getKpsn(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->kpsn, indices);
  };

  inline void setKps(const cv::Mat &kps) { this->kps = kps; };
  inline void setKpsn(const cv::Mat &kpsn) { this->kpsn = kpsn; }
  inline void setOctave(const cv::Mat &octave) { this->octave = octave; }
  inline void setSigma2(const cv::Mat &sigma2) { this->sigma2 = sigma2; }
  inline void setSigma2inv(const cv::Mat &sigma2inv)
  {
    this->sigma2inv = sigma2inv;
  }

  // inline Tmat getTwc() const { return this->Twc; };
  inline const cv::Mat &getTwc() const { return this->Twc; };
  // inline void setTwc(const cv::Mat &Rwc, const cv::Mat &twc) { Twc.setRt(Rwc,
  // twc); };
  inline void setTwc(const cv::Mat &Rwc, const cv::Mat &twc)
  {
    Rwc.copyTo(this->Twc(cv::Rect(0, 0, 3, 3)));  // x y w h
    twc.copyTo(this->Twc(cv::Rect(3, 0, 1, 3)));  // x y w h
  };

  // inline void setTwc(const Tmat &Twc) { this->Twc = Twc; };
  inline void setTwc(const cv::Mat &Twc) { this->Twc = Twc; };
  inline cv::Mat getPmat() const
  {
    cv::Mat TwcInv = this->Twc.inv();
    TwcInv = TwcInv(cv::Rect(0, 0, 4, 3));
    return TwcInv;
  }
  inline Eigen::Matrix4d getTwcEigen() const
  {
    Eigen::Matrix4d TwcEigen;
    cv::cv2eigen(this->Twc, TwcEigen);
    return TwcEigen;
  }

  inline void applyDepthScale(double depthScale)
  {
    this->Twc(cv::Rect(3, 0, 1, 3)) *= depthScale;
  };

  const cv::Mat &getOctave() const { return octave; };
  cv::Mat getOctave(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->octave, indices);
  };
  const cv::Mat getSigma2inv(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->sigma2inv, indices);
  };
  const cv::Mat getSigma2(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->sigma2, indices);
  };

  const cv::Mat get_descriptor(const cv::Mat &indices) const;
  const cv::Mat &getDescriptors() const
  {
    // cv::Mat ret;
    // mDescriptors.copyTo(ret);
    return this->mDescriptors;
  };
  void setDescriptor(const cv::Mat &mDescriptors)
  {
    this->mDescriptors = mDescriptors;
  };
  std::vector<cv::Mat> get_bow_feature() const
  {
    std::vector<cv::Mat> ret;
    ret.resize(mDescriptors.rows);
    for (size_t i = 0; mDescriptors.rows > i; ++i)
    {
      ret[i] = mDescriptors.row(i);
    }
    return ret;
  }

  inline const cv::Mat get_idx_tracked_points() const
  {
    return this->idx_tracked_points;
  };
  inline const cv::Mat get_idx_tracked_points(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->mDescriptors, indices);
  };
  inline void set_idx_tracked_points(const cv::Mat &idx_tracked_points)
  {
    this->idx_tracked_points = idx_tracked_points;
  };

  // const cv::Mat get_P() const;
  // void project_word2camera(const cv::Mat &points3d, cv::Mat &kpsn) const;
  // void project_camera2image(
  //     const cv::Mat &points3d, cv::Mat &kpsi, cv::Mat &depth) const;
  void projectFrame2World(const cv::Mat &kpsFrame, cv::Mat &kpsWorld) const;
  void projectWorld2Frame(const cv::Mat &kpsWorld, cv::Mat &kpsFrame) const;
  void projectWorld2Frame(
      const cv::Mat &kpsWorld, cv::Mat &kpsFrame, cv::Mat &depth) const;
  // void projectWorld2Image(const cv::Mat &kpsWorld, cv::Mat &kpsImage) const;
  // void projectWorld2Image(const cv::Mat &kpsWorld, cv::Mat &kpsImage, cv::Mat
  // &depth) const; void projectWorld2Image(
  //     const cv::Mat &points4d,
  //     cv::Mat &normalized_plane_points,
  //     cv::Mat &depth) const
  // {
  //   normalized_plane_points =
  //       (this->Twc.inverse().to_cvmat() * points4d.t()).t();
  //   normalized_plane_points = normalized_plane_points.colRange(0, 3);
  //   depth = normalized_plane_points.col(2);
  // }

  // inline void project(
  //     const cv::Mat points3d, cv::Mat &normalized_plane_points) const
  // {
  //   normalized_plane_points = (this->Twc.to_cvmat() * points3d.t()).t();
  //   normalized_plane_points = normalized_plane_points.colRange(0, 3);
  // }

  inline void addObservation(MapPointID mapPointID, ImgPointIdx imgPointIdx)
  {
    observedPoints[mapPointID] = imgPointIdx;
  }
  inline ImgPointIdx getObservation(MapPointID mapPointID)
  {
    auto it = observedPoints.find(mapPointID);
    return it == observedPoints.end() ? -1 : it->second;
  }
  inline void removeObservation(MapPointID mapPointID)
  {
    observedPoints.erase(mapPointID);
  }
};

// inline const cv::Mat Key_frame::get_P() const
// {
// cv::Mat P;
// cv::hconcat(this->Rwc, this->twc, P);
// return this->Twc.get(cv::Rect(0, 0, 4, 3));
// }
