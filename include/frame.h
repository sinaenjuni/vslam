#pragma once

#include <opencv2/opencv.hpp>

class Camera;
class Frame
{
 public:
  int id;
  // cv::Mat img;

  // cv::Mat sigma2;
  // cv::Mat sigma2inv;
  // A stereo image informations.
  // cv::Mat kps_r;
  // cv::Mat kpsn_r;
  // cv::Mat descriptor_r;
  // cv::Mat octave_r;
  // cv::Mat sigma2_r;
  // cv::Mat sigma2inv_r;

 private:
  // KeyPoints, normalized KeyPoints and, descriptors
  cv::Mat kps;
  cv::Mat kpsn;
  cv::Mat descriptor;
  cv::Mat octave;

  cv::Mat Twc = cv::Mat::eye(4, 4, CV_64F);

 public:
  Frame();
  Frame(Camera &camera, std::vector<cv::KeyPoint> &pts, cv::Mat &desc);
  ~Frame();

  const int get_id() const;
  // a front const means that not change output value at calling area
  // a back const means guarantee that not change the member variables inside
  // this function.
  const cv::Mat &get_kps() const;
  const cv::Mat get_kps(const cv::Mat &indices) const;
  const cv::Mat &get_kpsn() const;
  const cv::Mat get_kpsn(const cv::Mat &indices) const;
  const cv::Mat &get_octave() const;
  const cv::Mat get_octave(const cv::Mat &indices) const;
  const cv::Mat get_sigma2inv(const cv::Mat &indices) const;
  const cv::Mat get_sigma2(const cv::Mat &indices) const;
  const cv::Mat get_descriptor(const cv::Mat &indices) const;
  const cv::Mat &get_descriptor() const;

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

inline const int Frame::get_id() const { return id; }

inline const cv::Mat &Frame::get_kps() const { return kps; }

inline const cv::Mat &Frame::get_kpsn() const { return kpsn; }

inline const cv::Mat &Frame::get_octave() const { return octave; }
inline const cv::Mat &Frame::get_descriptor() const { return this->descriptor; }
inline void Frame::set_Twc(const cv::Mat &Rwc, const cv::Mat &twc)
{
  Rwc.copyTo(this->Twc(cv::Rect(0, 0, 3, 3)));
  twc.copyTo(this->Twc(cv::Rect(3, 0, 1, 3)));
}

inline const cv::Mat Frame::get_Twc() { return this->Twc; }

inline const cv::Mat Frame::get_P() const
{
  cv::Mat P;
  // cv::hconcat(this->Rwc, this->twc, P);
  return this->Twc(cv::Rect(0, 0, 4, 3));
}
