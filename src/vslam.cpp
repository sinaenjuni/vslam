#include "vslam.h"

#include "key_frame.h"
#include "misc.h"

VSLAM::VSLAM()
{
  this->settings = prase_settings("../settings/KITTI_Monocular.yaml");
  this->settings.print();

  this->camera = Camera(settings);
  this->feature_extractor = new FAST_ORB_extractor(settings);
  this->feature_matcher = new BFMatcher(cv::BFMatcher::create(cv::NORM_HAMMING, false));
}

VSLAM::~VSLAM() {}

void VSLAM::run()
{
  cv::VideoCapture cap(settings.img_path);
  cap.set(cv::CAP_PROP_CONVERT_RGB, 0);

  Key_frame* last_frame = nullptr;
  cv::Mat img, view_img;
  while (true)
  {
    cap.read(img);
    if (img.rows <= 0 && img.cols <= 0)
    {
      break;
    }
    cv::cvtColor(img, view_img, cv::COLOR_GRAY2BGR);

    Key_frame* frame = new Key_frame(img, camera, feature_extractor);

    if (frame->get_nkps() < 1000)
    {
      PRINT(frame->get_id());
      continue;
    }

    if (last_frame == nullptr)
    { 
      last_frame = frame;
      continue;
    }

    cv::Mat idx_match_cur, idx_match_ref;
    this->feature_matcher->compute_match(frame, last_frame, idx_match_cur, idx_match_ref);

    // cv::Mat Rwc, twc;
    // Geometry::pose_estimation_with_essential_matrix(
    //     frame, last_frame, idx_match_cur, idx_match_ref, Rwc, twc);
    // frame->set_Twc(Rwc, twc);

    // Misc::draw_kps(view_img, kps);
    Misc::draw_kps(view_img, frame->get_kps());
    Misc::draw_line(
        view_img, Matrix::slice_cvmat(frame->get_kps(), idx_match_cur),
        Matrix::slice_cvmat(last_frame->get_kps(), idx_match_ref));
    cv::imshow("image", view_img);

    last_frame = frame;

    // int key = cv::waitKey(1e3/30);
    int key = cv::waitKey(1);
    if (key == 32) key = cv::waitKey();  // Space
    if (key == 27) break;                // ESC
  }
}
