#include "geometry.h"

#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

#include "camera.h"
#include "entities.h"
#include "frame.h"
#include "map_point.h"
#include "misc.h"
#include "settings.h"

namespace Geometry
{
// Calculate T(transformation matrix) from a points2 to a points1
// resultly, can get a T12 matrix
// void estimatePoseWithEssentialMatrix(
//     KeyFramePtr keyFrame1,
//     KeyFramePtr keyFrame2,
//     cv::Mat &idx_match1,
//     cv::Mat &idx_match2,
//     cv::Mat &R,
//     cv::Mat &t)
// {
//   cv::Mat points1(keyFrame1->getKpsn(idx_match1));
//   cv::Mat points2(keyFrame2->getKpsn(idx_match2));
//   // cv::Mat points1 = Matrix::slice_cvmat(kpsn1, idx_match1);
//   // cv::Mat points2 = Matrix::slice_cvmat(kpsn2, idx_match2);

//   cv::Mat E, inlier;
//   // Calculate a transformation matrix from the points1 to the points`2.
//   E = cv::findEssentialMat(
//       points1,
//       points2,
//       1.0,
//       cv::Point2d(0, 0),
//       //  cv::RANSAC, 0.999, 0.0004, inlier);
//       cv::RANSAC,
//       0.999,
//       0.004,
//       inlier);
//   // PRINT(E);

//   cv::recoverPose(E, points1, points2, R, t, 1.0, cv::Point2d(0, 0), inlier);
//   // PRINT(R);
//   // PRINT(t);

//   cv::Mat inlier1(0, 0, CV_16U), inlier2(0, 0, CV_16U);
//   for (size_t i = 0; i < inlier.rows; i++)
//   {
//     if (inlier.at<uchar>(i) == 1)
//     {
//       inlier1.push_back(idx_match1.row(i));
//       inlier2.push_back(idx_match2.row(i));
//     }
//   }

//   inlier1.copyTo(idx_match1);
//   inlier2.copyTo(idx_match2);
// }

// std::vector<MapPointPtr> triangulate(
//     const KeyFramePtr keyFrame1,
//     const KeyFramePtr keyFrame2,
//     const cv::Mat &indMask1,
//     const cv::Mat &indMask2,
//     const Camera &camera,
//     const Settings &settings)
// {
//   cv::Mat points4d;
//   cv::triangulatePoints(
//       keyFrame1->getPmat(),
//       keyFrame2->getPmat(),
//       keyFrame1->getKpsn(indMask1).t(),
//       keyFrame2->getKpsn(indMask2).t(),
//       points4d);
//   points4d = points4d.t();
//   // points4d = points4d / cv::repeat(points4d.col(3), 1, 4);
//   cv::Mat w = points4d.col(3);
//   points4d.col(0) /= w;
//   points4d.col(1) /= w;
//   points4d.col(2) /= w;
//   points4d.col(3) = 1.0;

//   // calculate reprojection points
//   cv::Mat kpsReprojected1, depth1;
//   keyFrame1->projectWorld2Frame(points4d, kpsReprojected1, depth1);
//   camera.project(kpsReprojected1, kpsReprojected1);
//   cv::Mat kpsReprojected2, depth2;
//   keyFrame2->projectWorld2Frame(points4d, kpsReprojected2, depth2);
//   camera.project(kpsReprojected2, kpsReprojected2);

//   // check negative depth
//   cv::Mat bad_depth1 = depth1 < 0;
//   cv::Mat bad_depth2 = depth2 < 0;

//   cv::Mat kpsMatched1 = keyFrame1->getKps(indMask1);
//   cv::Mat kpsMatched2 = keyFrame2->getKps(indMask2);

//   // check reprojection errors
//   cv::Mat reprojectionError1 = kpsReprojected1 - kpsMatched1;
//   cv::reduce(reprojectionError1, reprojectionError1, 1, cv::REDUCE_SUM2);
//   reprojectionError1 =
//       reprojectionError1.mul(keyFrame1->getSigma2inv(indMask1));

//   cv::Mat reprojectionError2 = kpsReprojected2 - kpsMatched2;
//   cv::reduce(reprojectionError2, reprojectionError2, 1, cv::REDUCE_SUM2);
//   reprojectionError2 =
//       reprojectionError2.mul(keyFrame2->getSigma2inv(indMask2));

//   cv::Mat bad_reprojection_cur =
//       reprojectionError1 > settings.chi_square_threshold;
//   cv::Mat bad_reprojection_ref =
//       reprojectionError2 > settings.chi_square_threshold;

//   // check cos parallex
//   cv::Mat ray1, normRayPerRow1, normRay1;
//   keyFrame1->projectFrame2World(keyFrame1->getKpsn(indMask1), ray1);
//   // cv::reduce(ray1, normRayPerRow1, 1, cv::REDUCE_SUM2);
//   // cv::sqrt(normRayPerRow1, normRayPerRow1);
//   // cv::repeat(normRayPerRow1, 1, 3, normRay1);
//   // cv::divide(ray1, normRay1, ray1);
//   cv::normalize(ray1, ray1, 1.0, 0.0, cv::NORM_L2, -1, cv::noArray());

//   cv::Mat ray2, normRayPerRow2, normRay2;
//   keyFrame2->projectFrame2World(keyFrame2->getKpsn(indMask2), ray2);
//   // cv::reduce(ray2, normRayPerRow2, 1, cv::REDUCE_SUM2);
//   // cv::sqrt(normRayPerRow2, normRayPerRow2);
//   // cv::repeat(normRayPerRow2, 1, 3, normRay2);
//   // cv::divide(ray2, normRay2, ray2);
//   cv::normalize(ray2, ray2, 1.0, 0.0, cv::NORM_L2, -1, cv::noArray());

//   cv::Mat cosParallax;
//   cv::reduce(ray1.mul(ray2), cosParallax, 1, cv::REDUCE_SUM);
//   cv::Mat bad_cos_parallax =
//       (cosParallax < 0) | (cosParallax > settings.cos_max_parallax);

//   // check scale consistency
//   double scale_consistency_ratio =
//       settings.scale_consistency_factor * settings.scaleFactor;
//   cv::Mat scaled_depth_cur = keyFrame1->getSigma2(indMask1).mul(depth1);
//   cv::Mat consistency_scaled_depth_cur =
//       scaled_depth_cur * scale_consistency_ratio;

//   cv::Mat scaled_depth_ref = keyFrame2->getSigma2(indMask2).mul(depth2);
//   cv::Mat consistency_scaled_depth_ref =
//       scaled_depth_ref * scale_consistency_ratio;

//   cv::Mat bad_scale_consistency =
//       (scaled_depth_cur > consistency_scaled_depth_ref) |
//       (scaled_depth_ref > consistency_scaled_depth_cur);

//   cv::Mat bad_points = bad_depth1 | bad_depth2 | bad_reprojection_cur |
//                        bad_reprojection_ref | bad_cos_parallax |
//                        bad_scale_consistency;

//   std::vector<MapPointPtr> ret;
//   for (size_t i = 0; i < points4d.rows; ++i)
//   {
//     if (bad_points.at<bool>(i))
//     {
//       continue;
//     }

//     cv::Vec4d point4d = points4d.row(i);

//     cv::Vec<uint8_t, 32> descriptor = keyFrame1->getDescriptors().row(i);
//     MapPointPtr mapPoint = std::make_shared<MapPoint>(
//         PosD{point4d.val[0], point4d.val[1], point4d.val[2]}, descriptor);

//     mapPoint->addObservation(keyFrame1, indMask1.at<size_t>(i));
//     mapPoint->addObservation(keyFrame2, indMask2.at<size_t>(i));

//     keyFrame1->addObservation(mapPoint->getID(), indMask1.at<size_t>(i));
//     keyFrame2->addObservation(mapPoint->getID(), indMask2.at<size_t>(i));

//     ret.push_back(mapPoint);
//   }

//   return ret;
// }

// double calcMedianDepth(KeyFramePtr keyFrame, std::vector<MapPointPtr>
// points4d)
// {
//   cv::Mat listOfPoint4d(points4d.size(), 4, CV_64F);
//   for (size_t i = 0; i < points4d.size(); ++i)
//   {
//     listOfPoint4d.at<double>(i, 0) = points4d[i]->getPos().x;
//     listOfPoint4d.at<double>(i, 1) = points4d[i]->getPos().y;
//     listOfPoint4d.at<double>(i, 2) = points4d[i]->getPos().z;
//     listOfPoint4d.at<double>(i, 3) = 1;
//   }

//   cv::Mat Twc2 = keyFrame->getTwc().row(2);
//   cv::Mat sortedDepth = (Twc2 * listOfPoint4d.t()).t();
//   cv::sort(sortedDepth, sortedDepth, cv::SORT_EVERY_COLUMN);
//   int n = sortedDepth.rows / 2;
//   double ret = sortedDepth.at<double>(n);
//   return ret;
// }
}  // namespace Geometry
