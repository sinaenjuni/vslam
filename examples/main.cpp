#include <opencv2/core/hal/interface.h>

#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "entities.h"
#include "extractor.h"
#include "geometry.h"
#include "matcher.h"
#include "misc.h"
#include "vslam.h"

cv::Scalar octaveToColor(int octave, int maxOctave = 8)
{
  int val = cvRound(255.0 * octave / maxOctave);
  cv::Mat gray(1, 1, CV_8UC1, cv::Scalar(val));
  cv::Mat color;
  cv::applyColorMap(gray, color, cv::COLORMAP_JET);
  cv::Vec3b col = color.at<cv::Vec3b>(0, 0);
  return cv::Scalar(col[0], col[1], col[2]);
}

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

  // Camera camera = Camera(settings);

  Map map(settings);
  Viewer3D viewer3d(settings, map);
  std::thread viewer_thread = viewer3d.runWithThread(viewer3d);

  // FastOrbExtractor *feature_extractor = new FastOrbExtractor(
  //     settings.nPoints,
  //     settings.nLevels,
  //     settings.imgWidth,
  //     settings.imgHeight,
  //     settings.scaleFactor,
  //     7,
  //     20);

  cv::Ptr<cv::ORB> feature_extractor = cv::ORB::create(
      settings.nPoints,
      settings.scaleFactor,
      settings.nLevels,
      31,
      0,
      2,
      cv::ORB::FAST_SCORE,
      31,
      20);

  Matcher *matcher = new Matcher(0.6, false);

  // IFeature_matcher *featrue_matcher =
  // new BFMatcher(cv::BFMatcher::create(cv::NORM_HAMMING, false));

  cv::VideoCapture cap(settings.imgPath);
  // cap.set(cv::CAP_PROP_CONVERT_RGB, 0);

  // Tracker tracker;
  KeyFramePtr lastKeyFrame = nullptr;
  bool is_init = false;
  cv::Mat grayImg, colorImg;
  KeyFrame *framePrev = nullptr;

  while (true)
  {
    bool readRet = cap.read(grayImg);
    if (readRet && grayImg.rows <= 0 && grayImg.cols <= 0)
    {
      break;
    }
    cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

    KeyFrame *frameCur = new KeyFrame(grayImg, feature_extractor);

    if (framePrev == nullptr)
    {
      framePrev = frameCur;
      continue;
    }

    // for (const cv::KeyPoint kp : kps)
    // {
    //   PRINT(kp.pt);
    // }
    // cv::Ptr<cv::BFMatcher> bfMatcher =
    //     cv::BFMatcher::create(cv::NORM_HAMMING, false);
    // std::vector<std::vector<cv::DMatch>> matches;
    // bfMatcher->knnMatch(
    //     frameCur->getDescriptors(), framePrev->getDescriptors(), matches, 2);

    // std::unordered_map<int, std::pair<int, float>> duplications(
    //     settings.nPoints);
    // std::vector<int> matchF1(settings.nPoints, -1);
    // std::vector<int> matchF2(settings.nPoints, -1);
    // int nMatches = 0;
    // for (const std::vector<cv::DMatch> &match : matches)
    // {
    //   const cv::DMatch &m0 = match[0];
    //   const cv::DMatch &m1 = match[1];

    //   if (m1.distance * 0.75 < m0.distance)
    //   {
    //     // The ratio test, remove points that are not
    //     // sufficiently close to the nearest neighbor.
    //     continue;
    //   }

    //   if (duplications.find(m0.trainIdx) == duplications.end())
    //   // trainIdx가 중복되지 않는경우
    //   {
    //     // idx_match_query.push_back(m0.queryIdx);
    //     // idx_match_train.push_back(m0.trainIdx);
    //     matchF1[m0.queryIdx] = m0.trainIdx;
    //     matchF2[m0.trainIdx] = m0.queryIdx;
    //     // matchF1 = (query frame index, train frame index)
    //     // matchF2 = (train frame index, query frame index)
    //     duplications[m0.trainIdx] = {m0.queryIdx, m0.distance};
    //     nMatches++;
    //   }
    //   else  // trainIdx가 중복되는 경우
    //   {
    //     if (m0.distance < duplications[m0.].second)
    //     // 이전 trainIdx 보다 해밍 거리가 가까우면 대체
    //     {
    //       int preQueryIdx = duplications[m0.trainIdx].first;
    //       matchF1[preQueryIdx] = m0.trainIdx;
    //       matchF2[m0.trainIdx] = m0.queryIdx;
    //       duplications[m0.trainIdx] = {m0.queryIdx, m0.distance};
    //     }
    //   }
    // }

    std::vector<int> matchPrev, matchCur;
    int nMatches = matcher->knnMatch(framePrev, frameCur, matchPrev, matchCur);
    PRINT(nMatches);

    if (nMatches < 100)
    {
      PRINT("Not enough matches, skipping frame.");
      delete frameCur;
      continue;
    }

    cv::Mat Rwc, twc;
    std::vector<bool> vbTriangulated;
    



    // for (size_t queryIdx = 0; queryIdx < nMatches; ++queryIdx)
    // {
    //   int trainIdx = matchCur[queryIdx];
    //   if (trainIdx == -1)
    //   {
    //     continue;
    //   }

    //   cv::Point pt1 = frameCur->getKps()[queryIdx].pt;
    //   cv::Point pt2 = framePrev->getKps()[trainIdx].pt;
    //   cv::line(colorImg, pt1, pt2, cv::Scalar(0, 0, 255), 4);
    // }

    // framePrev = frameCur;
    // cv::Mat
    // idx_match_query, idx_match_train; matcher->matching(
    //     frameCur->getDescriptors(),
    //     framePrev->getDescriptors(),
    //     idx_match_query,
    //     idx_match_train);
    // std::vector<cv::KeyPoint> kps = frame->getKps();
    // cv::Mat desc = frame->getDescriptors();
    // for (size_t i = 0; i < frame->getN(); ++i)
    // {
    //   PRINT(kps[i].size, kps[i].angle, desc.row(i));
    // }

    // std::vector<cv::KeyPoint> kps2;
    // cv::Ptr<cv::ORB> orb =
    //     cv::ORB::create(5000, 1.2f, 8, 19, 0, 2, cv::ORB::FAST_SCORE, 31,
    //     20);
    // orb->detect(grayImg, kps2);
    // orb->compute(grayImg, kps2, cv::noArray());
    // for (const cv::KeyPoint &pt : kps2)
    // {
    //   PRINT(pt.angle, pt.size);
    // }
    // cv::FAST(grayImg, kps2, 30, true);
    // std::vector<int> nPointsPerLevel(settings.nLevels, 0);
    for (size_t idxCur = 0; idxCur < matchCur.size(); ++idxCur)
    {
      // PRINT(kps2[i].size, kps2[i].angle, kps2[i].response,
      // kps2[i].octave);
      // nPointsPerLevel[kps2[i].octave] += 1;
      // cv::circle(
      //     colorImg,
      //     kps2[i].pt,
      //     2,
      //     octaveToColor(kps2[i].octave),
      //     -1,
      //     cv::LINE_AA);
      const int idxPrev = matchCur[idxCur];
      if (idxPrev == -1)
      {
        continue;
      }
      cv::Point pt1 = framePrev->getKps()[idxPrev].pt;
      cv::Point pt2 = frameCur->getKps()[idxCur].pt;
      cv::circle(colorImg, pt1, 2, cv::Scalar(0, 255, 0), 1);
      cv::circle(colorImg, pt2, 2, cv::Scalar(255, 0, 0), 1);
      cv::line(colorImg, pt1, pt2, cv::Scalar(0, 0, 255), 1);
    }
    // for (int &count : nPointsPerLevel)
    // {
    //   PRINT(count);
    // }
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
    // Misc::draw_kps(colorImg, framePrev->getKps(), cv::Scalar(255, 0, 0));
    // Misc::draw_kps(colorImg, frameCur->getKps(), cv::Scalar(0, 255, 0));
    // for (size_t i = 0; i < idx_match_query.rows; ++i)
    // {
    //   int idx_query = idx_match_query.at<int>(i, 0);
    //   int idx_train = idx_match_train.at<int>(i, 0);

    //   cv::Point2f pt1 = frameCur->getKps()[idx_query].pt;
    //   cv::Point2f pt2 = framePrev->getKps()[idx_train].pt;
    //   cv::line(colorImg, pt1, pt2, cv::Scalar(0, 255, 0), 1);
    // }
    // Misc::draw_line(colorImg, const cv::Mat kps_cur, const cv::Mat kps_ref)
    // Misc::draw_kps(colorImg, framePrev->getKps(), cv::Scalar(0, 255, 0));
    // Misc::draw_kps(colorImg, frameCur->getKps(), cv::Scalar(255, 0, 0));
    cv::imshow("image", colorImg);

    // int key = cv::waitKey(1e3/30);
    int key = cv::waitKey();
    if (key == 32) key = cv::waitKey();  // Space
    if (key == 27) break;                // ESC

    // lastKeyFrame = keyFrame;
    framePrev = frameCur;
  }

  cv::destroyAllWindows();
  viewer_thread.join();
}

int main()
{
  vslam_main();
  return 0;
}