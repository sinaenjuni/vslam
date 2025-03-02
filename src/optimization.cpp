// #include "optimization.h"

// namespace Optimization
// {
// namespace Filter
// {
// cv::Mat filtering_points4d(
//     const Key_frame *const frame1,
//     const Key_frame *const frame2,
//     const Camera &camera,
//     const Settings &settings,
//     const cv::Mat &idx_frame1,
//     const cv::Mat &idx_frame2,
//     const cv::Mat &points4d)
// {
//   cv::Mat reprojection_kps_cur, depth_cur;
//   frame1->project(points4d, reprojection_kps_cur, depth_cur);
//   cv::Mat reprojection_kps_ref, depth_ref;
//   frame2->project(points4d, reprojection_kps_ref, depth_ref);

//   // check negative depth
//   cv::Mat bad_depth_cur = depth_cur < 0;
//   cv::Mat bad_depth_ref = depth_ref < 0;

//   cv::Mat matched_kps_cur = frame1->get_kps(idx_frame1);
//   cv::Mat matched_kps_ref = frame2->get_kps(idx_frame2);

//   // check reprojection errors
//   cv::Mat reprojection_error_cur = reprojection_kps_cur - matched_kps_cur;
//   cv::reduce(reprojection_error_cur, reprojection_error_cur, 1, cv::REDUCE_SUM2);
//   reprojection_error_cur = reprojection_error_cur.mul(frame1->get_sigma2inv(idx_frame1));

//   cv::Mat reprojection_error_ref = reprojection_kps_ref - matched_kps_ref;
//   cv::reduce(reprojection_error_ref, reprojection_error_ref, 1, cv::REDUCE_SUM2);
//   reprojection_error_ref = reprojection_error_ref.mul(frame2->get_sigma2inv(idx_frame2));

//   cv::Mat bad_reprojection_cur = reprojection_error_cur > settings.kChi2Mono;
//   cv::Mat bad_reprojection_ref = reprojection_error_ref > settings.kChi2Mono;

//   // check cos parallex
//   cv::Mat ray_cur, ray_norm_pre_row_cur, ray_norm_cur;
//   frame1->project_camera2world(frame1->get_kpsn(idx_frame1), ray_cur);
//   cv::reduce(ray_cur, ray_norm_pre_row_cur, 1, cv::REDUCE_SUM2);
//   cv::sqrt(ray_norm_pre_row_cur, ray_norm_pre_row_cur);
//   cv::repeat(ray_norm_pre_row_cur, 1, 3, ray_norm_cur);
//   cv::divide(ray_cur, ray_norm_cur, ray_cur);

//   cv::Mat ray_ref, ray_norm_pre_row_ref, ray_norm_ref;
//   frame1->project_camera2world(frame1->get_kpsn(idx_frame2), ray_ref);
//   cv::reduce(ray_ref, ray_norm_pre_row_ref, 1, cv::REDUCE_SUM2);
//   cv::sqrt(ray_norm_pre_row_ref, ray_norm_pre_row_ref);
//   cv::repeat(ray_norm_pre_row_ref, 1, 3, ray_norm_ref);
//   cv::divide(ray_ref, ray_norm_ref, ray_ref);

//   cv::Mat cos_parallax;
//   cv::reduce(ray_cur.mul(ray_ref), cos_parallax, 1, cv::REDUCE_SUM);
//   cv::Mat bad_cos_parallax = (cos_parallax < 0) | (cos_parallax > settings.cos_max_parallax);

//   // check scale consistency
//   double scale_consistency_ratio = settings.scale_consistency_factor * settings.scale_factor;
//   cv::Mat scaled_depth_cur = frame1->get_sigma2(idx_frame1).mul(depth_cur);
//   cv::Mat consistency_scaled_depth_cur = scaled_depth_cur * scale_consistency_ratio;

//   cv::Mat scaled_depth_ref = frame1->get_sigma2(idx_frame2).mul(depth_ref);
//   cv::Mat consistency_scaled_depth_ref = scaled_depth_ref * scale_consistency_ratio;

//   cv::Mat bad_scale_consistency = (scaled_depth_cur > consistency_scaled_depth_ref) |
//                                   (scaled_depth_ref > consistency_scaled_depth_cur);

//   cv::Mat bad_points = bad_depth_cur | bad_depth_ref | bad_reprojection_cur |
//   bad_reprojection_ref |
//                        bad_cos_parallax | bad_scale_consistency;
// }
// }  // namespace Filter
// }  // namespace Optimization