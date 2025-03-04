#pragma once

#include "entities.h"

class Map_point
{
 private:
  PosF position;
  // std::array<uint16_t, 32> descripor;
  cv::Vec<uint8_t, 32> descripor;

 public:
  Map_point();
  Map_point(PosF position, cv::Vec<uint8_t, 32> descripor);
  ~Map_point();
  inline PosF get_position() { return position; };
  
};
