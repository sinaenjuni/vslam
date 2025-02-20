#pragma once

// #include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>

class Octree_node {
private:
  int xmin, ymin, xmax, ymax;
  std::vector<cv::KeyPoint> kps;
  bool is_done;

public:
  Octree_node(int xmin, int ymin, int xmax, int ymax);
  ~Octree_node();

  void add_point(cv::KeyPoint kp);
  bool get_done() const;
  std::array<std::optional<Octree_node>, 4> divide_node();
  size_t get_kps_size() const;
  cv::KeyPoint get_good_kp() const;
  bool operator<(const Octree_node &other) const;
  void draw_node(cv::Mat &img, cv::Scalar color, int thickness);
  friend std::ostream &operator<<(std::ostream &os, const Octree_node &node);
};

inline bool Octree_node::get_done() const { return is_done; }

inline size_t Octree_node::get_kps_size() const { return kps.size(); }

inline void Octree_node::add_point(cv::KeyPoint kp) {
  if (kp.pt.x < xmin || kp.pt.x >= xmax || kp.pt.y < ymin || kp.pt.y >= ymax) {
    throw std::invalid_argument("Invalid Key points.");
    std::exit(EXIT_FAILURE);
  }
  kps.push_back(kp);
}

namespace Octree {

inline bool compare_response(cv::KeyPoint &A, cv::KeyPoint &B) {
  // if a return value that this compare function is true, the data A and,
  // B exchange order but if not the data order not change.
  return (A.response > B.response ? true : false);
}

void rectify_kps(std::vector<cv::KeyPoint> &kps, int n_points, int xmin,
                 int ymin, int xmax, int ymax);

} // namespace Octree