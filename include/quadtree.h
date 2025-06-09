#pragma once

// #include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>

class QuadTreeNode
{
 private:
  int xmin, ymin, xmax, ymax;
  std::vector<cv::KeyPoint> kps;
  bool is_done;

 public:
  QuadTreeNode(int xmin, int ymin, int xmax, int ymax);
  ~QuadTreeNode();

  bool isDone() const;
  bool operator<(const QuadTreeNode &other) const;
  void addKeyPoint(cv::KeyPoint kp);
  void draw_node(cv::Mat &img, cv::Scalar color, int thickness);
  size_t getNKps() const;
  cv::KeyPoint getGoodKeyPoint() const;
  std::array<std::optional<QuadTreeNode>, 4> divideNode();
  friend std::ostream &operator<<(std::ostream &os, const QuadTreeNode &node);
};

inline bool QuadTreeNode::isDone() const { return is_done; };

inline size_t QuadTreeNode::getNKps() const { return kps.size(); };

inline void QuadTreeNode::addKeyPoint(cv::KeyPoint kp)
{
  if (kp.pt.x < xmin || kp.pt.x >= xmax || kp.pt.y < ymin || kp.pt.y >= ymax)
  {
    throw std::invalid_argument("Invalid Key points.");
    std::exit(EXIT_FAILURE);
  }
  kps.push_back(kp);
};

class QuadTree
{
 private:
  static inline bool compareResponse(cv::KeyPoint &A, cv::KeyPoint &B)
  {
    // if a return value that this compare function is true, the data A and,
    // B exchange order but if not the data order not change.
    return (A.response > B.response ? true : false);
  }

 public:
  static std::vector<cv::KeyPoint> rectifyKps(
      const std::vector<cv::KeyPoint> &kps,
      const int n_points,
      const int xmin,
      const int ymin,
      const int xmax,
      const int ymax);
};