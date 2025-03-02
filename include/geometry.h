#include <opencv2/opencv.hpp>

class Key_frame;

namespace Geometry
{
void pose_estimation_with_essential_matrix(
    const cv::Mat &kpsn1,
    const cv::Mat &kpsn2,
    cv::Mat &idx_match1,
    cv::Mat &idx_match2,
    cv::Mat &R,
    cv::Mat &t);

cv::Mat triangulate(
    Key_frame const *const frame1,
    Key_frame const *const frame2,
    const cv::Mat idx_match1,
    const cv::Mat idx_match2);
}  // namespace Geometry