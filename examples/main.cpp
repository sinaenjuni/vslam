#include "vslam.h"

int main()
{
  Viewer3D viewer3d;
  viewer3d.setup();
  std::thread viewer_thread = viewer3d.run_with_thread(viewer3d);

  VSLAM vslam;
  vslam.run();

  viewer3d.stop_thread();
  viewer_thread.join();
  return 0;
}