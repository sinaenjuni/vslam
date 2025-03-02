#include "tracker.h"

#include <opencv2/core/hal/interface.h>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

#include "feature.h"
#include "geometry.h"
#include "key_frame.h"
#include "misc.h"
#include "settings.h"

Tracker::Tracker() {}
Tracker::Tracker(Settings settings, IFeature_matcher *matcher)
    : max_size(settings.max_tracking_size), matcher(matcher)
{
}

Rt Tracker::track(Key_frame *key_frame, const cv::Mat &view_img)
{
  Rt Twc;
  if (this->tracked_frames.size() == 0)
  {
    tracked_frames.push(key_frame);
    Misc::draw_kps(view_img, key_frame->get_kps());
    return Twc;
  }

  // descriptors.(key_frame->get_descriptor());
  // cv::(key_frame->get_descriptor(), descriptors);
  // PRINT(descriptors.size());

  Key_frame *target_frame = tracked_frames.back();
  cv::Mat target_decriptor;
  if (target_frame->get_idx_tracked_points().rows == 0)
  {
    target_decriptor = target_frame->get_descriptor();
  }
  else
  {
    target_decriptor = target_frame->get_descriptor(target_frame->get_idx_tracked_points());
  }

  cv::Mat idx_frame1, idx_frame2;
  matcher->compute_match(key_frame->get_descriptor(), target_decriptor, idx_frame1, idx_frame2);

  cv::Mat Rwc, twc;
  Geometry::pose_estimation_with_essential_matrix(
      key_frame->get_kpsn(), target_frame->get_kpsn(), idx_frame1, idx_frame2, Rwc, twc);
  Twc.setRt(Rwc, twc);
  Twc = target_frame->get_Twc() * Twc;

  if (idx_frame1.rows < 100)
  {
    key_frame->set_idx_tracked_points(idx_frame1);
  }
  target_frame->set_idx_tracked_points(idx_frame2);

  tracked_frames.push(key_frame);

  Misc::draw_kps(view_img, key_frame->get_kps(idx_frame1));
  return Twc;
}