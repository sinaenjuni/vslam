#include "vslam.h"

#include "frame.h"

VSLAM::VSLAM()
{
  this->settings = prase_settings("../settings/KITTI_Monocular.yaml");
  this->settings.print();

  this->camera = Camera(
      this->settings.fx, this->settings.fy, this->settings.cx, this->settings.cy, this->settings.bf,
      this->settings.width, this->settings.height);

  this->feature_extractor = new FAST_ORB_extractor(
      this->settings.n_points, this->settings.n_levels, this->settings.width, this->settings.height,
      this->settings.scale2_factors, this->settings.scale2inv_factors);

  this->feature_matcher = new BFMatcher(cv::BFMatcher::create(cv::NORM_HAMMING, false));
}

VSLAM::~VSLAM() {}

void VSLAM::run()
{
  cv::VideoCapture cap(settings.img_path);
  cap.set(cv::CAP_PROP_CONVERT_RGB, 0);

  Frame *last_frame = nullptr;
  cv::Mat img, view_img;
  while (true)
  {
    cap.read(img);
    if (img.rows <= 0 && img.cols <= 0)
    {
      break;
    }
    cv::cvtColor(img, view_img, cv::COLOR_GRAY2BGR);

    // feature extract step
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    feature_extractor->detect_and_compute(img, kps, desc);

    if (kps.size() < 1000)
    {
      continue;
    }

    Frame *frame = new Frame(camera, kps, desc);
    if (last_frame == nullptr)
    {
      last_frame = frame;
      continue;
    }

    // cv::Mat idx_match_cur, idx_match_ref;
    // this->feature_matcher->feature_matching(
    //     frame, last_frame, idx_match_cur, idx_match_ref);

    // cv::Mat Rwc, twc;
    // Geometry::pose_estimation_with_essential_matrix(
    //     frame, last_frame, idx_match_cur, idx_match_ref, Rwc, twc);
    // frame->set_Twc(Rwc, twc);

    Misc::draw_kps(view_img, kps);
    cv::imshow("image", view_img);
    // int key = cv::waitKey(1e3/30);
    int key = cv::waitKey(1);
    if (key == 32) key = cv::waitKey();  // Space
    if (key == 27) break;                // ESC
  }
}
