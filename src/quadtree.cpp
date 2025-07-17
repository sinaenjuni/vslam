#include "quadtree.h"

#include <opencv2/core/types.hpp>

QuadTreeNode::QuadTreeNode(int xmin, int ymin, int xmax, int ymax)
    : xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax), is_done(false)
{
  if ((xmax - xmin) < 3 || (ymax - ymin) < 3 || kps.size() == 1)
  {
    is_done = true;
  }
}

QuadTreeNode::~QuadTreeNode() {}

std::array<std::optional<QuadTreeNode>, 4> QuadTreeNode::divideNode()
{
  int xhalf = xmin + ((xmax - xmin) / 2);
  int yhalf = ymin + ((ymax - ymin) / 2);

  std::optional<QuadTreeNode> UL;
  std::optional<QuadTreeNode> UR;
  std::optional<QuadTreeNode> BL;
  std::optional<QuadTreeNode> BR;

  for (const cv::KeyPoint &kp : kps)
  {
    if (kp.pt.x >= xmin && kp.pt.x < xhalf && kp.pt.y >= ymin &&
        kp.pt.y < yhalf)
    {
      if (!UL) UL.emplace(xmin, ymin, xhalf, yhalf);
      UL->addKeyPoint(kp);
    }
    else if (
        kp.pt.x >= xhalf && kp.pt.x < xmax && kp.pt.y >= ymin &&
        kp.pt.y < yhalf)
    {
      if (!UR) UR.emplace(xhalf, ymin, xmax, yhalf);
      UR->addKeyPoint(kp);
    }
    else if (
        kp.pt.x >= xmin && kp.pt.x < xhalf && kp.pt.y >= yhalf &&
        kp.pt.y < ymax)
    {
      if (!BL) BL.emplace(xmin, yhalf, xhalf, ymax);
      BL->addKeyPoint(kp);
    }
    else if (
        kp.pt.x >= xhalf && kp.pt.x < xmax && kp.pt.y >= yhalf &&
        kp.pt.y < ymax)
    {
      if (!BR) BR.emplace(xhalf, yhalf, xmax, ymax);
      BR->addKeyPoint(kp);
    }
  }

  return {UL, UR, BL, BR};
}

cv::KeyPoint QuadTreeNode::getGoodKeyPoint() const
{
  cv::KeyPoint ret(kps[0]);
  for (size_t i = 1; i < kps.size(); i++)
  {
    if (ret.response < kps[i].response)
    {
      ret = kps[i];
    }
  }
  return ret;
}

bool QuadTreeNode::operator<(const QuadTreeNode &other) const
{
  return this->getNKps() < other.getNKps();  // it is for descending
}

void QuadTreeNode::draw_node(
    cv::Mat &img, cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1)
{
  cv::rectangle(
      img,
      cv::Point(xmin, ymin),
      cv::Point(xmax, ymax),
      color,
      thickness,
      cv::LINE_AA);
  for (auto &kp : kps)
  {
    cv::circle(img, kp.pt, 2, color, -1, cv::LINE_AA);
  }
}

std::ostream &operator<<(std::ostream &os, const QuadTreeNode &node)
// overload <<(insert) operator for printing object information.
{
  os << "Octree_node: ["
     << "xmin: " << node.xmin << ", ymin: " << node.ymin
     << ", xmax: " << node.xmax << ", ymax: " << node.ymax
     << ", keypoints: " << node.getNKps()
     << ", is_done: " << (node.isDone() ? "true" : "false") << "]";
  return os;
}

std::vector<cv::KeyPoint> QuadTree::rectifyKps(
    const std::vector<cv::KeyPoint> &kps,
    const int nPoints,
    const int xmin,
    const int ymin,
    const int xmax,
    const int ymax)
{
  const int width = xmax - xmin;
  const int height = ymax - ymin;
  const int nInitNodes = std::ceil(static_cast<float>(width) / (height));
  // W / H = nInitNodes, ex) 1200 / 400 = 3
  const int hX = width / nInitNodes;
  // W / nInitNodes = hX, ex) 1200 / 3 = 400

  std::vector<QuadTreeNode> lNodes;
  int xStart = xmin;
  for (int i = 0; i < nInitNodes; ++i)
  {
    // QuadTreeNode node = QuadTreeNode(hX * i, 0, hX * (i + 1), ymax);
    // lNodes.push_back(node);
    int xEnd = (i == nInitNodes - 1) ? xmax : xStart + hX;
    lNodes.emplace_back(hX * i, ymin, xEnd, ymax);
    xStart = xEnd;
  }

  for (const cv::KeyPoint &kp : kps)
  {
    int idx = std::min(static_cast<int>((kp.pt.x - xmin) / hX), nInitNodes - 1);
    lNodes[idx].addKeyPoint(kp);
  }
  // for (size_t i = 0; i < kps.size(); i++)
  // {
  //   lNodes[static_cast<int>(kps[i].pt.x / hX)].addKeyPoint(kps[i]);
  // }

  std::priority_queue<QuadTreeNode> pqNodes;
  // sort by descending
  // for (size_t i = 0; i < nInitNodes; i++)
  // {
  //   pqNodes.push(lNodes[i]);
  // }
  for (const QuadTreeNode &node : lNodes)
  {
    if (node.getNKps() == 0)
    {
      continue;
    }
    pqNodes.push(node);
  }
  lNodes.clear();

  while (!pqNodes.empty())
  {
    if (nPoints <=
        static_cast<int>(pqNodes.size()) + static_cast<int>(lNodes.size()))
    // Check the number of q_nodes and l_nodes to stop this iteration.
    {
      break;
    }

    QuadTreeNode node = pqNodes.top();
    pqNodes.pop();

    if (node.isDone())
    {
      lNodes.push_back(node);  // node that state is done store to l_nodes.
      continue;
    }

    if (node.getNKps() > 2)
    // if node have more that 2 keypoints divide to child nodes.
    {
      for (std::optional<QuadTreeNode> &node : node.divideNode())
      {
        if (node)
        {
          pqNodes.push(*node);
        }
      }
    }
    else  // if node have only one keypoint, store to l_nodes.
    {
      lNodes.push_back(node);
    }
  }

  while (!pqNodes.empty())
  {
    lNodes.push_back(pqNodes.top());
    pqNodes.pop();
  }

  std::vector<cv::KeyPoint> ret;
  for (size_t i = 0; i < lNodes.size(); i++)
  {
    ret.push_back(lNodes[i].getGoodKeyPoint());
  }

  std::sort(ret.begin(), ret.end(), QuadTree::compareResponse);
  if (static_cast<int>(ret.size()) > nPoints)
  {
    ret = std::vector<cv::KeyPoint>(ret.begin(), ret.begin() + nPoints);
    // ret.resize(nPoints);
  }

  return ret;
}