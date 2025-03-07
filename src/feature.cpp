#include "feature.h"

#include <opencv2/core/hal/interface.h>

#include "camera.h"
#include "key_frame.h"
#include "octree.h"

FAST_ORB_extractor::FAST_ORB_extractor(Settings settings, Camera *camera)
    : n_points(settings.n_points),
      n_levels(settings.n_levels),
      width(settings.width),
      height(settings.height),
      scale2_factors(settings.scale2_factors),
      scale2inv_factors(settings.scale2inv_factors),
      camera(camera),
      descriptor(cv::ORB::create())
{
  // reserve() function doesn't set the vector size.
  img_pyramid.resize(n_levels);
  img_pyramid[0] = cv::Mat::zeros(height, width, CV_8UC1);
  for (size_t i = 1; i < n_levels; i++)
  {
    // resize using scale factor
    cv::resize(
        img_pyramid[0], img_pyramid[i], cv::Size(), scale2inv_factors[i], scale2inv_factors[i]);
  }
}

FAST_ORB_extractor::FAST_ORB_extractor(
    int n_points,
    int n_levels,
    int width,
    int height,
    std::vector<double> &scale2_factors,
    std::vector<double> &scale2inv_factors)
    : n_points(n_points),
      n_levels(n_levels),
      width(width),
      height(height),
      scale2_factors(scale2_factors),
      scale2inv_factors(scale2inv_factors),
      descriptor(cv::ORB::create())
{
  // reserve() function doesn't set the vector size.
  img_pyramid.resize(n_levels);
  img_pyramid[0] = cv::Mat::zeros(height, width, CV_8UC1);
  for (size_t i = 1; i < n_levels; i++)
  {
    // resize using scale factor
    cv::resize(
        img_pyramid[0], img_pyramid[i], cv::Size(), scale2inv_factors[i], scale2inv_factors[i]);
  }
}

FAST_ORB_extractor::~FAST_ORB_extractor() {}

void FAST_ORB_extractor::detect_with_octave(
    const cv::Mat &img, std::vector<cv::KeyPoint> &kps, int octave)
{
  // std::vector<cv::KeyPoint> kps;
  cv::resize(
      img, img_pyramid[octave], cv::Size(), scale2inv_factors[octave], scale2inv_factors[octave]);
  // detector->detect(img_pyramid[octave], kps);
  cv::FAST(img_pyramid[octave], kps, 20, true);

  for (size_t i = 0; i < kps.size(); i++)
  {
    kps[i].octave = octave;
    kps[i].pt = kps[i].pt * scale2_factors[octave];
    kps[i].size = 30;
  }
}

void FAST_ORB_extractor::detect(const cv::Mat &img, std::vector<cv::KeyPoint> &kps)
{
  for (size_t i = 0; i < n_levels; i++)
  {
    std::vector<cv::KeyPoint> octave_kps;
    detect_with_octave(img, octave_kps, i);
    kps.insert(kps.end(), octave_kps.begin(), octave_kps.end());
  }
  Octree::rectify_kps(kps, n_points, 0, 0, width, height);
}

void FAST_ORB_extractor::detect_and_compute(
    const cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Mat &desc)
{
  detect(img, kps);
  descriptor->compute(img, kps, desc);
}

bool FAST_ORB_extractor::extract(
    const cv::Mat &img,
    cv::Mat &kps,
    cv::Mat &kpsn,
    cv::Mat &descriptor,
    cv::Mat &octave,
    cv::Mat &sigma2,
    cv::Mat &sigma2inv)
{
  // Key_frame *key_frame = new Key_frame();
  std::vector<cv::KeyPoint> pts;
  // cv::Mat descriptor;
  this->detect_and_compute(img, pts, descriptor);
  // key_frame->set_descriptor(descriptor);

  if (pts.size() < 1000)
  {
    return false;
  }

  int nkps = pts.size();
  kps = cv::Mat(nkps, 2, CV_64F);
  kpsn = cv::Mat(nkps, 2, CV_64F);
  octave = cv::Mat(nkps, 1, CV_16F);
  sigma2 = cv::Mat(nkps, 1, CV_64F);
  sigma2inv = cv::Mat(nkps, 1, CV_64F);

  for (size_t i = 0; i < nkps; i++)
  {
    kps.at<double>(i, 0) = pts[i].pt.x;
    kps.at<double>(i, 1) = pts[i].pt.y;

    octave.at<uint8_t>(i) = pts[i].octave;

    sigma2.at<double_t>(i) = this->get_scale2(pts[i].octave);
    sigma2inv.at<double_t>(i) = this->get_scale2inv(pts[i].octave);
  }
  this->camera->unproject(kps, kpsn);
  return true;

  // key_frame->get_kps().create(nkps, 2, CV_64F);
  // key_frame->set_kps(kps, kpsn, octave, sigma2, sigma2inv);

  // return key_frame;
}

double_t FAST_ORB_extractor::get_scale2inv(const int octave) const
{
  return this->scale2inv_factors[octave];
}

double_t FAST_ORB_extractor::get_scale2(const int octave) const
{
  return this->scale2_factors[octave];
}

BFMatcher::BFMatcher(cv::Ptr<cv::BFMatcher> matcher) : matcher(matcher) {}
BFMatcher::~BFMatcher() {}
void BFMatcher::matching(
    const cv::Mat &query, const cv::Mat &train, cv::Mat &idx_match_query, cv::Mat &idx_match_train)
{
  std::vector<std::vector<cv::DMatch>> matches;
  matcher->knnMatch(query, train, matches, 2);
  // PRINT(query->get_descriptor().size(), train->get_descriptor().size(), matches.size());
  // cv::Mat idx_cur(0, 0, CV_16U), idx_ref(0, 0, CV_16U);
  idx_match_query.create(0, 0, CV_16U);
  idx_match_train.create(0, 0, CV_16U);
  std::unordered_map<int, std::pair<int, int>> duplications;
  // 현재 영상에 존재하는 포인트를 기준으로 이전 영상에서 유사한 포인트를
  // 찾는다. 이때, 현재 영상에 존재하는 포인트는 이전 영상에 존재하는 포인트
  // 여러개에 연결될 수 있다.
  for (std::vector<cv::DMatch> &match : matches)
  {
    // A m0 is a nearest neighbor and a m1 is a next ones.
    // Find two nearest points based on a query descriptor.
    const cv::DMatch &m0 = match[0];
    const cv::DMatch &m1 = match[1];

    // PRINT(m0.distance, m1.distance*0.75);
    if (m0.distance > m1.distance * 0.75)
    {
      // 가장 가까운 포인트보다 두 번째로 가까운 포인트의
      // (거리 * 0.75) 보다 작은 포인트는 버린다.
      continue;
    }
    if (duplications.find(m0.trainIdx) == duplications.end())
    {
      idx_match_query.push_back(m0.queryIdx);
      idx_match_train.push_back(m0.trainIdx);
      duplications[m0.trainIdx] = {idx_match_train.rows - 1, m0.distance};
    }
    else
    {
      if (duplications[m0.trainIdx].second > m0.distance)
      {
        int target_idx = duplications[m0.trainIdx].first;
        idx_match_query.row(target_idx) = m0.queryIdx;
        idx_match_train.row(target_idx) = m0.trainIdx;
        duplications[m0.trainIdx] = {target_idx, m0.distance};
      }
    }
  }

  if (idx_match_query.rows != idx_match_train.rows)
  {
    throw std::length_error("Not valid index length.");
  }
}
