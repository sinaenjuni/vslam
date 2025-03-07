#pragma once

#include <opencv2/core/mat.hpp>

#include "key_frame.h"

class Settings;

namespace Geometry
{
void estimatePoseWithEssentialMatrix(
    KeyFramePtr keyFrame1,
    KeyFramePtr keyFrame2,
    cv::Mat &idx_match1,
    cv::Mat &idx_match2,
    cv::Mat &R,
    cv::Mat &t);

std::vector<MapPointPtr> triangulate(
    const KeyFramePtr frame1,
    const KeyFramePtr frame2,
    const cv::Mat &idx_match1,
    const cv::Mat &idx_match2,
    const Settings settings);
}  // namespace Geometry