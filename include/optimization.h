#include <opencv2/opencv.hpp>

class Key_frame;
class Camera;
class Settings;

namespace Optimization
{
namespace Filter
{
cv::Mat filtering_points4d(
    const Key_frame *const frame1,
    const Key_frame *const frame2,
    const Camera &camera,
    const Settings &settings,
    const cv::Mat &idx_frame1,
    const cv::Mat &idx_frame2,
    const cv::Mat &points4d);

}
};  // namespace Optimization