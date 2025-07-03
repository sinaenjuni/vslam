#pragma once

#include <opencv2/core/mat.hpp>

#include "frame.h"

class Settings;
class Camera;

namespace Geometry
{
void estimatePoseWithEssentialMatrix(
    KeyFrame *keyFrame1,
    KeyFrame *keyFrame2,
    cv::Mat &idx_match1,
    cv::Mat &idx_match2,
    cv::Mat &R,
    cv::Mat &t);

std::vector<MapPoint *> triangulate(
    const KeyFrame *keyFrame1,
    const KeyFrame *keyFrame2,
    const cv::Mat &indMask1,
    const cv::Mat &indMask2,
    const Camera &camera,
    const Settings &settings);

double calcMedianDepth(KeyFrame *keyFrame, std::vector<MapPoint *> points4d);
}  // namespace Geometry
