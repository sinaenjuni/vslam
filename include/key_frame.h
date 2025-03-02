#pragma once

#include "entities.h"
#include "misc.h"

class Camera;
class FAST_ORB_extractor;
class Key_frame
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

 public:
  Key_frame();
  Key_frame(const cv::Mat &img, Camera &camera, FAST_ORB_extractor *feature_extractor);
  Key_frame(Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat &desc);
  ~Key_frame();

  inline const int get_id() const { return this->id; };
  // a front const means that not change output value at calling area
  // a back const means guarantee that not change the member variables inside
  // this function.
  inline const int get_nkps() const { return this->kps.rows; };

  inline const cv::Mat &get_kps() const { return this->kps; };
  inline const cv::Mat get_kps(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->kps, indices);
  };

  inline const cv::Mat &get_kpsn() const { return this->kpsn; };
  inline const cv::Mat get_kpsn(const cv::Mat &indices) const
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

  inline Rt get_Twc() const { return this->Twc; };
  inline void set_Twc(const cv::Mat &Rwc, const cv::Mat &twc) { Twc.setRt(Rwc, twc); };
  inline void set_Twc(const Rt &Twc) { this->Twc = Twc; };

  inline void project(
      const cv::Mat points4d, cv::Mat &normalized_plane_points, cv::Mat &depth) const
  {
    normalized_plane_points = (this->Twc.inverse().to_cvmat() * points4d.t()).t();
    normalized_plane_points = normalized_plane_points.colRange(0, 3);
    depth = normalized_plane_points.col(2);
  }

  inline void project(const cv::Mat points3d, cv::Mat &normalized_plane_points) const
  {
    normalized_plane_points = (this->Twc.to_cvmat() * points3d.t()).t();
    normalized_plane_points = normalized_plane_points.colRange(0, 3);
  }

  const cv::Mat &get_octave() const { return octave; };
  const cv::Mat get_octave(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->octave, indices);
  };
  const cv::Mat get_sigma2inv(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->sigma2inv, indices);
  };
  const cv::Mat get_sigma2(const cv::Mat &indices) const
  {
    return Matrix::slice_cvmat(this->sigma2, indices);
  };

  const cv::Mat get_descriptor(const cv::Mat &indices) const;
  const cv::Mat &get_descriptor() const { return this->descriptor; };
  void set_descriptor(const cv::Mat &descriptor) { this->descriptor = descriptor; };

  inline const cv::Mat get_idx_tracked_points() const { return this->idx_tracked_points; };
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
  // void project_camera2world(const cv::Mat &kpsn, cv::Mat &kpsw);
  // void project2image_coordinates(const cv::Mat &points4d, cv::Mat &kps,
  // cv::Mat &depth) const;
};

// inline const cv::Mat Key_frame::get_P() const
// {
// cv::Mat P;
// cv::hconcat(this->Rwc, this->twc, P);
// return this->Twc.get(cv::Rect(0, 0, 4, 3));
// }
