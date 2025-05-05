#include <opencv2/core/hal/interface.h>

#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include "entities.h"
#include "feature.h"
#include "geometry.h"
#include "misc.h"
#include "vslam.h"

void vslam_main()
{
  Settings settings =
      Settings::prase_settings("../settings/KITTI_Monocular.yaml");
  settings.print();

  KeyFrame::initStaticVariables(
      settings.imgWidth,
      settings.imgHeight,
      settings.fx,
      settings.fy,
      settings.cx,
      settings.cy,
      settings.p1,
      settings.p2,
      settings.k1,
      settings.k2);

  Camera camera = Camera(settings);

  Map map(settings);
  Viewer3D viewer3d(settings, map);
  std::thread viewer_thread = viewer3d.runWithThread(viewer3d);

  FastOrbExtractor *feature_extractor = new FastOrbExtractor(
      settings.nPoints,
      settings.nLevels,
      settings.imgWidth,
      settings.imgHeight,
      settings.scaleFactor,
      20);

  IFeature_matcher *featrue_matcher =
      new BFMatcher(cv::BFMatcher::create(cv::NORM_HAMMING, false));

  cv::VideoCapture cap(settings.imgPath);
  // cap.set(cv::CAP_PROP_CONVERT_RGB, 0);

  // Tracker tracker;
  KeyFramePtr lastKeyFrame = nullptr;
  bool is_init = false;
  cv::Mat grayImg, colorImg;
  while (true)
  {
    bool readRet = cap.read(grayImg);
    if (readRet && grayImg.rows <= 0 && grayImg.cols <= 0)
    {
      break;
    }
    cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

    KeyFrame *frame = new KeyFrame(grayImg, feature_extractor);

    // KeyFramePtr keyFrame = std::make_shared<KeyFrame>();
    // bool is_ok = feature_extractor->extract(grayIMG, keyFrame);

    // if (!is_ok)
    // {
    //   PRINT("This image doesn't have sufficient key points.");
    //   continue;
    // }
    // if (lastKeyFrame == nullptr)
    // {
    //   lastKeyFrame = keyFrame;
    //   continue;
    // }

    // if (is_init == false)
    // {
    //   cv::Mat indMatchedKeyFrame, indMatchedLastKeyFrame;
    //   featrue_matcher->matching(
    //       keyFrame->getDescriptor(),
    //       lastKeyFrame->getDescriptor(),
    //       indMatchedKeyFrame,
    //       indMatchedLastKeyFrame);

    //   cv::Mat Rwc, twc;
    //   Geometry::estimatePoseWithEssentialMatrix(
    //       keyFrame,
    //       lastKeyFrame,
    //       indMatchedKeyFrame,
    //       indMatchedLastKeyFrame,
    //       Rwc,
    //       twc);
    //   keyFrame->setTwc(Rwc, twc);

    //   std::vector<MapPointPtr> points4d = Geometry::triangulate(
    //       keyFrame,
    //       lastKeyFrame,
    //       indMatchedKeyFrame,
    //       indMatchedLastKeyFrame,
    //       camera,
    //       settings);

    //   map.addkeyFrame(lastKeyFrame);
    //   map.addkeyFrame(keyFrame);

    //   double medianDepth = Geometry::calcMedianDepth(keyFrame, points4d);
    //   double desiredMedianDepth = 10;
    //   double scaleMedianDepth = desiredMedianDepth / medianDepth;
    //   keyFrame->applyDepthScale(scaleMedianDepth);

    //   for (MapPointPtr mapPoint : points4d)
    //   {
    //     mapPoint->applyDepthScale(scaleMedianDepth);
    //     map.addMapPoint(mapPoint);
    //   }
    //   is_init = true;
    // }
    // else
    // {
    //   // keyFrame->setTwc(lastKeyFrame->getTwc());
    //   // map.addkeyFrame(keyFrame);
    // }

    // PRINT(lastKeyFrame->getTwc());

    // PRINT(frame->getKps());
    Misc::draw_kps(colorImg, frame->getKps());
    cv::imshow("image", colorImg);

    // int key = cv::waitKey(1e3/30);
    int key = cv::waitKey();
    if (key == 32) key = cv::waitKey();  // Space
    if (key == 27) break;                // ESC

    // lastKeyFrame = keyFrame;
  }

  cv::destroyAllWindows();
  viewer_thread.join();
}

int main()
{
  vslam_main();
  return 0;
}