#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <iostream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>

class KeyFrame;
class MapPoint;

using MapPointID = int;
using KeyFrameID = int;
using ImgPointIdx = size_t;

using KeyFramePtr = std::shared_ptr<KeyFrame>;
using MapPointPtr = std::shared_ptr<MapPoint>;

struct PosD
{
  double x, y, z;

  friend std::ostream &operator<<(std::ostream &os, const PosD &PosD)
  {
    os << "PosD\n" << PosD.x << ", " << PosD.y << ", " << PosD.z << "\n";
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