#include "map_point.h"

Map_point::Map_point() {}
Map_point::Map_point(PosF position, cv::Vec<uint8_t, 32> descripor)
    : position(position), descripor(descripor)
{
}
Map_point::~Map_point() {}