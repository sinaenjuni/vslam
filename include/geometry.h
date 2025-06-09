#pragma once

#include <opencv2/core/mat.hpp>

#include "frame.h"

class Settings;
class Camera;

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
    const KeyFramePtr keyFrame1,
    const KeyFramePtr keyFrame2,
    const cv::Mat &indMask1,
    const cv::Mat &indMask2,
    const Camera &camera,
    const Settings &settings);

double calcMedianDepth(KeyFramePtr keyFrame, std::vector<MapPointPtr> points4d);
}  // namespace Geometry
