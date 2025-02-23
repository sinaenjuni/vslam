#pragma once

#include <opencv2/opencv.hpp>
class Camera;
class IFeature_extractor;
class Key_frame
{
 private:
  int id = -1;
  // KeyPoints, normalized KeyPoints and, descriptors
  cv::Mat kps;
  cv::Mat kpsn;
  cv::Mat descriptor;
  cv::Mat octave;
  cv::Mat Twc = cv::Mat::eye(4, 4, CV_64F);

 public:
  Key_frame();
  Key_frame(const cv::Mat &img, Camera &camera, IFeature_extractor *feature_extractor);
  Key_frame(Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat &desc);
  ~Key_frame();

  inline const int get_id() const { return this->id; };
  // a front const means that not change output value at calling area
  // a back const means guarantee that not change the member variables inside
  // this function.
  inline const int get_nkps() const { return this->kps.rows; };

  const cv::Mat &get_kps() const { return this->kps; };
  const cv::Mat get_kps(const cv::Mat &indices) const;
  const cv::Mat &get_kpsn() const;
  const cv::Mat get_kpsn(const cv::Mat &indices) const;
  const cv::Mat &get_octave() const;
  const cv::Mat get_octave(const cv::Mat &indices) const;
  const cv::Mat get_sigma2inv(const cv::Mat &indices) const;
  const cv::Mat get_sigma2(const cv::Mat &indices) const;
  const cv::Mat get_descriptor(const cv::Mat &indices) const;
  const cv::Mat &get_descriptor() const { return this->descriptor; }

  void set_Twc(const cv::Mat &Rwc, const cv::Mat &twc);
  const cv::Mat get_Twc();
  const cv::Mat get_P() const;
  void project(const cv::Mat &points4d, cv::Mat &kps, cv::Mat &depth) const;
  void project_word2camera(const cv::Mat &points3d, cv::Mat &kpsn) const;
  // void project_camera2image(
  //     const cv::Mat &points3d, cv::Mat &kpsi, cv::Mat &depth) const;
  // void project_camera2world(const cv::Mat &kpsn, cv::Mat &kpsw);
  // void project2image_coordinates(const cv::Mat &points4d, cv::Mat &kps,
  // cv::Mat &depth) const;
};

inline const cv::Mat &Key_frame::get_kpsn() const { return kpsn; }
inline const cv::Mat &Key_frame::get_octave() const { return octave; }

inline void Key_frame::set_Twc(const cv::Mat &Rwc, const cv::Mat &twc)
{
  Rwc.copyTo(this->Twc(cv::Rect(0, 0, 3, 3)));
  twc.copyTo(this->Twc(cv::Rect(3, 0, 1, 3)));
}

inline const cv::Mat Key_frame::get_Twc() { return this->Twc; }

inline const cv::Mat Key_frame::get_P() const
{
  cv::Mat P;
  // cv::hconcat(this->Rwc, this->twc, P);
  return this->Twc(cv::Rect(0, 0, 4, 3));
}
