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
    : lastKeyFrame(nullptr), matcher(matcher)
{
}

Rt Tracker::track(KeyFramePtr keyFrame, const cv::Mat &view_img)
{
  Rt Twc;
  if (this->lastKeyFrame == nullptr)
  {
    lastKeyFrame = keyFrame;
    Misc::draw_kps(view_img, keyFrame->getKps());
    return Twc;
  }

  cv::Mat descriptor_last_frame;
  if (lastKeyFrame->get_idx_tracked_points().rows == 0)
  {
    descriptor_last_frame = lastKeyFrame->getDescriptor();
  }
  else
  {
    descriptor_last_frame =
        lastKeyFrame->get_descriptor(lastKeyFrame->get_idx_tracked_points());
  }

  cv::Mat idx_key_frame, idx_last_frame;
  matcher->matching(
      keyFrame->getDescriptor(), descriptor_last_frame, idx_key_frame,
      idx_last_frame);

  cv::Mat Rwc, twc;
  Geometry::estimatePoseWithEssentialMatrix(
      keyFrame, lastKeyFrame, idx_key_frame, idx_last_frame, Rwc, twc);
  Twc.setRt(Rwc, twc);
  Twc = lastKeyFrame->getTwc() * Twc;

  if (idx_key_frame.rows < 100)
  {
    keyFrame->set_idx_tracked_points(idx_key_frame);
  }
  lastKeyFrame->set_idx_tracked_points(idx_last_frame);

  // tracked_frames.push(key_frame);

  Misc::draw_kps(view_img, keyFrame->getKps(idx_key_frame));
  return Twc;
}