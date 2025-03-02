#pragma once
#include <pangolin/pangolin.h>

#include <vector>

class Map;

class Viewer3D
{
 private:
  std::string window_name;
  int window_width;
  int window_height;
  int ViewpointX;
  int ViewpointY;
  float ViewpointZ;
  int ViewpointF;

  Map *map;
  //   std::vector<Key_frame> key_frames;
  //   std::vector<Map_point> map_points;
  std::atomic<bool> is_running;

 public:
  Viewer3D();
  ~Viewer3D();
  void setup(
      std::string window_name = "Visual slam",
      int win_width = 1024,
      int win_height = 768,
      int ViewpointX = 0,
      int ViewpointY = -100,
      float ViewpointZ = -0.1,
      int ViewpointF = 2000);
  inline void set_map(Map *map) { this->map = map; };
  void run();
  std::thread run_with_thread(Viewer3D &viewer3d);
  void stop_thread();
  inline bool get_is_running() { return this->is_running; }
  //   inline void set_key_frames()
  //   {
  //     key_frames.push_back(Key_frame_DO{
  //         // clang-format off
  //         (Eigen::Matrix4d() << 1, 0, 0, 0,
  //                               0, 1, 0, 0,
  //                               0, 0, 1, 0,
  //                               0, 0, 0, 1).finished(),
  //         Color{1.0, 0.0, 0.0}});
  //     // clang-format on

  //     key_frames.push_back(Key_frame_DO{
  //         (Eigen::Matrix4d() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 10, 0, 0, 0, 1).finished()
  //         // Color(0.0, 0.0, 1.0)
  //     });

  //     key_frames.push_back(Key_frame_DO{
  //         (Eigen::Matrix4d() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 20, 0, 0, 0, 1).finished()
  //         // Color(0.0, 0.0, 1.0)
  // });
  // }
};
