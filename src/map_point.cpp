#include "map_point.h"

int getMapPointID()
{
  static int _MAP_POINT_ID = -1;
  return ++_MAP_POINT_ID;
}

MapPoint::MapPoint() : id(getMapPointID()) {}
// MapPoint::MapPoint(PosD position, cv::Vec<uint8_t, 32> descriptor)
//     : id(getMapPointID()), position(position), descriptor(descriptor)
// {
// }
MapPoint::MapPoint(PosD position, cv::Vec<uint8_t, 32> descriptor) : MapPoint()
{
  this->position = position;
  this->descriptor = descriptor;
}
MapPoint::~MapPoint() {}