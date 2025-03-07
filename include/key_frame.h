#pragma once

#include <opencv2/core/mat.hpp>
#include <unordered_map>

#include "entities.h"
#include "misc.h"

class Camera;
class FAST_ORB_extractor;
class KeyFrame
{
 private:
  int id = -1;
  // KeyPoints, normalized KeyPoints and, descriptors
  cv::Mat kps;
  cv::Mat kpsn;
  cv::Mat descriptor;
  cv::Mat octave;
  cv::Mat idx_tracked_points;
  Rt Twc;

  cv::Mat sigma2;
  cv::Mat sigma2inv;
  Camera *camera;

  std::unordered_map<MapPointID, ImgPointIdx> observedPoints;

 public:
  KeyFrame();
  KeyFrame(
      cv::Mat &kps,
      cv::Mat &kpsn,
      cv::Mat &descriptor,
      cv::Mat &octave,
      cv::Mat &sigma2,
      cv::Mat &sigma2inv,
      Camera *camera);
  KeyFrame(
      const cv::Mat &img,
      Camera &camera,
      FAST_ORB_extractor *feature_extractor);
  KeyFrame(Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat &desc);
  ~KeyFrame();

  inline const int getID() const { return this->id; };
  // a front const means that not change output value at calling area
  // a back const means guarantee that not change the member variables inside
  // this function.
  inline const int get_nkps() const { return this->kps.rows; };

  inline const cv::Mat &getKps() const { return this->kps; };
  inline const cv::Mat getKps(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->kps, indices);
  };

  inline const cv::Mat &get_kpsn() const { return this->kpsn; };
  inline const cv::Mat getKpsn(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->kpsn, indices);
  };
  inline void set_kps(
      const cv::Mat &kps,
      const cv::Mat &kpsn,
      const cv::Mat &octave,
      const cv::Mat &sigma2,
      const cv::Mat &sigma2inv)
  {
    this->kps = kps;
    this->kpsn = kpsn;
    this->octave = octave;
    this->sigma2 = sigma2;
    this->sigma2inv = sigma2inv;
  };

  inline Rt getTwc() const { return this->Twc; };
  inline void setTwc(const cv::Mat &Rwc, const cv::Mat &twc)
  {
    Twc.setRt(Rwc, twc);
  };
  inline void setTwc(const Rt &Twc) { this->Twc = Twc; };
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
  const cv::Mat &getDescriptor() const
  {
    // cv::Mat ret;
    // descriptor.copyTo(ret);
    return this->descriptor;
  };
  void set_descriptor(const cv::Mat &descriptor)
  {
    this->descriptor = descriptor;
  };
  std::vector<cv::Mat> get_bow_feature() const
  {
    std::vector<cv::Mat> ret;
    ret.resize(descriptor.rows);
    for (size_t i = 0; descriptor.rows > i; ++i)
    {
      ret[i] = descriptor.row(i);
    }
    return ret;
  }

  inline const cv::Mat get_idx_tracked_points() const
  {
    return this->idx_tracked_points;
  };
  inline const cv::Mat get_idx_tracked_points(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->descriptor, indices);
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
  void projectWorld2Image(const cv::Mat &kpsWorld, cv::Mat &kpsImage) const;
  void projectWorld2Image(
      const cv::Mat &kpsWorld, cv::Mat &kpsImage, cv::Mat &depth) const;
  // void projectWorld2Image(
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
