#include "octree.h"

Octree_node::Octree_node(int xmin, int ymin, int xmax, int ymax)
    : xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax), is_done(false) {
  if ((xmax - xmin) < 3 || (ymax - ymin) < 3 || kps.size() == 1) {
    is_done = true;
  }
}

Octree_node::~Octree_node() {}

std::array<std::optional<Octree_node>, 4> Octree_node::divide_node() {
  int xhalf = xmin + ((xmax - xmin) / 2);
  int yhalf = ymin + ((ymax - ymin) / 2);

  std::optional<Octree_node> UL;
  std::optional<Octree_node> UR;
  std::optional<Octree_node> BL;
  std::optional<Octree_node> BR;

  for (auto kp : kps) {
    if (kp.pt.x >= xmin && kp.pt.x < xhalf && kp.pt.y >= ymin &&
        kp.pt.y < yhalf) {
      if (!UL)
        UL.emplace(xmin, ymin, xhalf, yhalf);
      UL->add_point(kp);
    } else if (kp.pt.x >= xhalf && kp.pt.x < xmax && kp.pt.y >= ymin &&
               kp.pt.y < yhalf) {
      if (!UR)
        UR.emplace(xhalf, ymin, xmax, yhalf);
      UR->add_point(kp);
    } else if (kp.pt.x >= xmin && kp.pt.x < xhalf && kp.pt.y >= yhalf &&
               kp.pt.y < ymax) {
      if (!BL)
        BL.emplace(xmin, yhalf, xhalf, ymax);
      BL->add_point(kp);
    } else if (kp.pt.x >= xhalf && kp.pt.x < xmax && kp.pt.y >= yhalf &&
               kp.pt.y < ymax) {
      if (!BR)
        BR.emplace(xhalf, yhalf, xmax, ymax);
      BR->add_point(kp);
    }
  }

  return {UL, UR, BL, BR};
}

cv::KeyPoint Octree_node::get_good_kp() const {
  cv::KeyPoint ret(kps[0]);
  for (size_t i = 1; i < kps.size(); i++) {
    if (ret.response < kps[i].response) {
      ret = kps[i];
    }
  }
  return ret;
}

bool Octree_node::operator<(const Octree_node &other) const {
  return this->get_kps_size() < other.get_kps_size(); // it is for decending
}

void Octree_node::draw_node(cv::Mat &img,
                            cv::Scalar color = cv::Scalar(0, 255, 0),
                            int thickness = 1) {
  cv::rectangle(img, cv::Point(xmin, ymin), cv::Point(xmax, ymax), color,
                thickness, cv::LINE_AA);
  for (auto &kp : kps) {
    cv::circle(img, kp.pt, 2, color, -1, cv::LINE_AA);
  }
}

std::ostream &operator<<(std::ostream &os, const Octree_node &node)
// overload <<(insert) operator for printing object information.
{
  os << "Octree_node: ["
     << "xmin: " << node.xmin << ", ymin: " << node.ymin
     << ", xmax: " << node.xmax << ", ymax: " << node.ymax
     << ", keypoints: " << node.get_kps_size()
     << ", is_done: " << (node.get_done() ? "true" : "false") << "]";
  return os;
}

namespace Octree {

void rectify_kps(std::vector<cv::KeyPoint> &kps, int n_points, int xmin,
                 int ymin, int xmax, int ymax) {
  int n_init_nodes = std::ceil((xmax - xmin) / (ymax - ymin));
  int hX = (xmax - xmin) / n_init_nodes;

  std::vector<Octree_node> l_nodes;
  for (size_t i = 0; i < n_init_nodes; i++) {
    Octree_node node = Octree_node(hX * i, 0, hX * (i + 1), ymax);
    l_nodes.push_back(node);
  }

  for (size_t i = 0; i < kps.size(); i++) {
    l_nodes[static_cast<int>(kps[i].pt.x / hX)].add_point(kps[i]);
  }

  std::priority_queue<Octree_node> q_nodes;
  for (size_t i = 0; i < n_init_nodes; i++) {
    q_nodes.push(l_nodes[i]);
  }
  l_nodes.clear();

  while (!q_nodes.empty()) {
    if (n_points <= q_nodes.size() + l_nodes.size())
    // Check the number of q_nodes and l_nodes to stop this iteration.
    {
      break;
    }

    Octree_node node = q_nodes.top();
    q_nodes.pop();

    if (node.get_done()) {
      l_nodes.push_back(node); // node that state is done store to l_nodes.
      // PRINT(l_nodes.size());
      continue;
    }

    if (node.get_kps_size() > 2) {
      for (auto &node : node.divide_node()) {
        if (!node)
          continue;
        q_nodes.push(*node);
      }
    }
  }

  while (!q_nodes.empty()) {
    l_nodes.push_back(q_nodes.top());
    q_nodes.pop();
  }

  std::vector<cv::KeyPoint> ret;
  for (size_t i = 0; i < l_nodes.size(); i++) {
    ret.push_back(l_nodes[i].get_good_kp());
  }
  std::sort(ret.begin(), ret.end(), compare_response);
  kps = std::vector<cv::KeyPoint>(ret.begin(), ret.begin() + n_points);
}
} // namespace Octree