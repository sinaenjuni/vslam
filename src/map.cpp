#include "map.h"

#include "camera.h"

// void Map::add_map_points(
//     const KeyFrame *const frame1,
//     const KeyFrame *const frame2,
//     const Camera &camera,
//     const cv::Mat &idx_frame1,
//     const cv::Mat &idx_frame2,
//     const cv::Mat &points4d)
// {
//   cv::Mat proj_kpsn1, proj_depth1, proj_kps1;
//   frame1->project(points4d, proj_kpsn1, proj_depth1);
//   camera.project(proj_kpsn1, proj_kps1);
//   // Misc::draw_kps(view_img, proj_kps1);

//   cv::Mat proj_kpsn2, proj_depth2, proj_kps2;
//   frame2->project(points4d, proj_kpsn2, proj_depth2);
//   camera.project(proj_kpsn2, proj_kps2);
//   // Misc::draw_kps(view_img, proj_kps2, cv::Scalar(255, 255, 0));

//   // check negative depth
//   cv::Mat bad_depth1 = proj_depth1 < 0;
//   cv::Mat bad_depth2 = proj_depth2 < 0;

//   cv::Mat matched_kps1 = frame1->getKps(idx_frame1);
//   cv::Mat matched_kps2 = frame2->getKps(idx_frame2);

//   // check reprojection errors
//   cv::Mat reprojection_error1 = proj_kps1 - frame1->getKps(idx_frame1);
//   cv::reduce(reprojection_error1, reprojection_error1, 1, cv::REDUCE_SUM2);
//   reprojection_error1 =
//       reprojection_error1.mul(frame1->getSigma2inv(idx_frame1));

//   cv::Mat reprojection_error2 = proj_kps2 - frame2->getKps(idx_frame2);
//   cv::reduce(reprojection_error2, reprojection_error2, 1, cv::REDUCE_SUM2);
//   reprojection_error2 =
//       reprojection_error2.mul(frame2->getSigma2inv(idx_frame2));

//   cv::Mat bad_reprojection_cur = reprojection_error1 > this->kChi;
//   cv::Mat bad_reprojection_ref = reprojection_error2 > this->kChi;

//   cv::Mat ray_cur, ray_norm_pre_row_cur, ray_norm_cur;
//   frame1->project(
//       Matrix::to_homogeneous(
//           Matrix::to_homogeneous(frame1->getKpsn(idx_frame1))),
//       ray_cur);
//   cv::reduce(ray_cur, ray_norm_pre_row_cur, 1, cv::REDUCE_SUM2);
//   cv::sqrt(ray_norm_pre_row_cur, ray_norm_pre_row_cur);
//   cv::repeat(ray_norm_pre_row_cur, 1, 3, ray_norm_cur);
//   cv::divide(ray_cur, ray_norm_cur, ray_cur);

//   cv::Mat ray_ref, ray_norm_pre_row_ref, ray_norm_ref;
//   frame2->project(
//       Matrix::to_homogeneous(
//           Matrix::to_homogeneous(frame2->getKpsn(idx_frame2))),
//       ray_ref);
//   cv::reduce(ray_ref, ray_norm_pre_row_ref, 1, cv::REDUCE_SUM2);
//   cv::sqrt(ray_norm_pre_row_ref, ray_norm_pre_row_ref);
//   cv::repeat(ray_norm_pre_row_ref, 1, 3, ray_norm_ref);
//   cv::divide(ray_ref, ray_norm_ref, ray_ref);

//   cv::Mat cos_parallax;
//   cv::reduce(ray_cur.mul(ray_ref), cos_parallax, 1, cv::REDUCE_SUM);
//   cv::Mat bad_cos_parallax =
//       (cos_parallax < 0) | (cos_parallax > this->cos_max_parallax);

//   // check scale consistency
//   double scale_consistency_ratio =
//       this->scale_consistency_factor * this->scale_factor;
//   cv::Mat scaled_depth_cur = frame1->get_sigma2(idx_frame1).mul(proj_depth1);
//   cv::Mat consistency_scaled_depth_cur =
//       scaled_depth_cur * scale_consistency_ratio;

//   cv::Mat scaled_depth_ref = frame2->get_sigma2(idx_frame2).mul(proj_depth2);
//   cv::Mat consistency_scaled_depth_ref =
//       scaled_depth_ref * scale_consistency_ratio;

//   cv::Mat bad_scale_consistency =
//       (scaled_depth_cur > consistency_scaled_depth_ref) |
//       (scaled_depth_ref > consistency_scaled_depth_cur);

//   cv::Mat bad_points = bad_depth1 | bad_depth2 | bad_reprojection_cur |
//                        bad_reprojection_ref | bad_cos_parallax |
//                        bad_scale_consistency;

//   cv::Mat descriptors = frame1->get_descriptor(idx_frame1);
//   for (size_t i = 0; i < points4d.rows; i++)
//   {
//     if (bad_points.at<bool>(i))
//     {
//       continue;
//     }
//     cv::Vec4d point4d = points4d.row(i);

//     cv::Vec<uint8_t, 32> descriptor = descriptors.row(i);
//     MapPointPtr mapPoint = std::make_shared<MapPoint>(
//         PosF{point4d.val[0], point4d.val[1], point4d.val[2]}, descriptor);
//     // print(descriptor);

//     // double_t x = matched_kps_cur.at<double_t>(i, 0);
//     // double_t y = matched_kps_cur.at<double_t>(i, 1);z

//     this->addMapPoint(mapPoint);
// }
// }