#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

class Rt
{
 private:
  cv::Mat mat;

 public:
  Rt() : mat(cv::Mat::eye(4, 4, CV_64F)) {}
  Rt(const Rt &other) : mat(other.mat.clone()) {}

  inline void setRt(const cv::Mat &Rwc, const cv::Mat &twc)
  {
    Rwc.copyTo(this->mat(cv::Rect(0, 0, 3, 3)));
    twc.copyTo(this->mat(cv::Rect(3, 0, 1, 3)));
  };

  inline Rt inverse() const
  {
    Rt ret(*this);
    ret.mat = ret.mat.inv();
    return ret;
  }

  inline cv::Mat to_cvmat() const { return this->mat; }

  inline Eigen::Matrix4d to_eigen() const
  {
    Eigen::Matrix4d eigen_mat;
    cv::cv2eigen(this->mat, eigen_mat);
    return eigen_mat;
  }

  inline Rt operator()(const int col, const int row) const
  {
    Rt ret(*this);
    ret.mat = ret.mat(cv::Rect(0, 0, col, row));
    return ret;
  }

  inline Rt operator*(Rt other)
  {
    Rt ret(*this);
    ret.mat = ret.mat * other.mat;
    return ret;
  };

  friend std::ostream &operator<<(std::ostream &os, const Rt &Rt)
  {
    os << Rt.mat << "\n";
    return os;
  }
};

struct PosF
{
  double x, y, z;

  friend std::ostream &operator<<(std::ostream &os, const PosF &PosF)
  {
    os << "PosF\n" << PosF.x << ", " << PosF.y << ", " << PosF.z << "\n";
    return os;
  }
};

struct Color
{
  float red = 1.0;
  float green = 1.0;
  float blue = 1.0;

  friend std::ostream &operator<<(std::ostream &os, const Color &color_dao)
  {
    os << "color_dao\n"
       << color_dao.red << ", " << color_dao.green << ", " << color_dao.blue << "\n";
    return os;
  }
};

struct Key_frame_DO
{
  Eigen::Matrix4d Twc;
  Color color;

  friend std::ostream &operator<<(std::ostream &os, const Key_frame_DO &key_frame_DO)
  {
    os << "key_frame_DO\n" << key_frame_DO.Twc << "\n" << key_frame_DO.color << "\n";
    return os;
  }
};

struct Map_point_DO
{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  Color color;

  friend std::ostream &operator<<(std::ostream &os, const Map_point_DO &Map_point_DO)
  {
    os << "Map_point_DO\n"
       << Map_point_DO.x << ", " << Map_point_DO.y << ", " << Map_point_DO.z << ", "
       << Map_point_DO.color << "\n";
    return os;
  }
};