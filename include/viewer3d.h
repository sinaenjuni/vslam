#pragma once

#include <pangolin/pangolin.h>

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
  void run();
  std::thread run_with_thread(Viewer3D &viewer3d);
  void stop_thread();
  inline bool get_is_running() { return this->is_running; }
};
