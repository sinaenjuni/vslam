#include "vslam.h"

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "entities.h"
#include "misc.h"

namespace VSLAM
{
void run()
{
  Settings settings = prase_settings("../settings/KITTI_Monocular.yaml");
  settings.print();

  IFeature_matcher* matcher = new BFMatcher(cv::BFMatcher::create(cv::NORM_HAMMING, false));
  Tracker tracker(settings, matcher);

  Map map(settings);
  Viewer3D viewer3d;
  viewer3d.setup();
  viewer3d.set_map(&map);
  std::thread viewer_thread = viewer3d.run_with_thread(viewer3d);

  Camera camera = Camera(settings);
  FAST_ORB_extractor* feature_extractor = new FAST_ORB_extractor(settings, &camera);

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

    // Key_frame* frame = new Key_frame(img, camera, feature_extractor);
    Key_frame* frame = feature_extractor->extract(img);
    if (frame->get_nkps() < 1000)
    {
      PRINT(frame->get_id());
      continue;
    }

    cv::Mat Rwc, twc;
    Rt Twc = tracker.track(frame, view_img);
    frame->set_Twc(Twc);

    map.add_key_frame(frame);

    // if (last_frame == nullptr)
    // {
    //   last_frame = frame;
    //   continue;
    // }

    // cv::Mat idx_match_cur, idx_match_ref;
    // matcher->compute_match(frame, last_frame, idx_match_cur, idx_match_ref);

    // cv::Mat Rwc, twc;
    // Geometry::pose_estimation_with_essential_matrix(
    //     frame, last_frame, idx_match_cur, idx_match_ref, Rwc, twc);
    // frame->set_Twc(Rwc, twc);

    // map.add_key_frame(last_frame);
    // map.add_key_frame(frame);

    // cv::Mat points4d = Geometry::triangulate(frame, last_frame, idx_match_cur, idx_match_ref);

    // PRINT(cv::repeat(projected_points.col(2), 1, 3));
    // PRINT(projected_points);
    // Misc::draw_kps(view_img, projected_points.colRange(0, 2), cv::Scalar(255, 255, 0));

    // cv::Mat test, test_depth;
    // camera.project((frame->get_Twc().inv() * points4d.t()).t(), test, test_depth);
    // PRINT((frame->get_Twc().inv() * points4d.t()).t().size());
    // PRINT((frame->get_Twc().inv() * points4d.t()).t());
    // map.add_map_points(frame, last_frame, idx_match_cur, idx_match_ref, points4d);

    // PRINT(points4d.size());
    // camera_coordinates = (last_frame->get_Twc() * points4d.t()).t();
    // projected_points = (camera.get_K() * camera_coordinates.colRange(0, 3).t()).t();
    // projected_points /= cv::repeat(projected_points.col(2), 1, 3);
    // Misc::draw_kps(view_img, projected_points.colRange(0, 2), cv::Scalar(255, 255, 255));

    // Misc::draw_kps(view_img, kps);
    // Misc::draw_kps(view_img, frame->get_kps());

    // Misc::draw_line(view_img, frame->get_kps(idx_match_cur), last_frame->get_kps(idx_match_ref));
    cv::imshow("image", view_img);

    // last_frame = frame;

    // int key = cv::waitKey(1e3/30);
    int key = cv::waitKey();
    if (key == 32) key = cv::waitKey();  // Space
    if (key == 27) break;                // ESC
  }

  cv::destroyAllWindows();
  viewer_thread.join();
}

}  // namespace VSLAM
