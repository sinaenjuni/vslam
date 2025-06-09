#include "viewer3d.h"

#include <pangolin/gl/gldraw.h>

#include <algorithm>
#include <mutex>
#include <vector>

#include "map.h"
#include "settings.h"

// Viewer3D::Viewer3D() : is_running(true)
// {
// Define Projection and initial ModelView matrix
// ProjectionMatrix arguemnts
// widht, height, ?, ?, widht//2, height//2, Near Clipping Plane, Far Clipping
// Plane
// pangolin::OpenGlRenderState s_cam(
//     pangolin::ProjectionMatrix(win_width, win_height, ViewpointF,
//     ViewpointF,
//                                win_width / 2, win_height / 2, 0.1, 1000),
//     pangolin::ModelViewLookAt(ViewpointX, ViewpointY, ViewpointZ, 0, 0, 0,
//                               0.0, -1.0, 0.0));

// Create Interactive View in window
// pangolin::Handler3D handler(s_cam);
// d_cam = pangolin::CreateDisplay()
//             .SetBounds(
//                 0.0, 1.0, 0.0, 1.0,
//                 -static_cast<double>(window_width) /
//                 static_cast<double>(window_height))
//             .SetHandler(&handler);

// key_frames.push_back(Key_frame_dao(
// (Eigen::Matrix4d() <<
//     1, 0, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 0,
//     0, 0, 0, 1).finished(),
//     Color(1.0, 0.0,0.0)
// ));

// key_frames.push_back(Key_frame_dao(
//   (Eigen::Matrix4d() <<
//       1, 0, 0, 0,
//       0, 1, 0, 0,
//       0, 0, 1, 10,
//       0, 0, 0, 1).finished()
//       // Color(0.0, 0.0, 1.0)
//   ));

// key_frames.push_back(Key_frame_dao(
//   (Eigen::Matrix4d() <<
//       1, 0, 0, 0,
//       0, 1, 0, 0,
//       0, 0, 1, 20,
//       0, 0, 0, 1).finished()
//       // Color(0.0, 0.0, 1.0)
//   ));

//   map_points.push_back(
// Map_point_dao(
//       1.0, 1.0, 1.0,
//         Color(0.0, 0.0, 1.0)
//     )
//   );

//   map_points.push_back(
//     Map_point_dao(
//       2.0, 1.0, 1.0
//     )
//   );

// this->key_frames.push_back(
//   (Eigen::Matrix4d() <<
//     1, 0, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 0,
//     0, 0, 0, 1).finished());

// this->key_frames.push_back(
//   (Eigen::Matrix4d() <<
//     1, 0, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 5,
//     0, 0, 0, 1).finished());
// }
Viewer3D::Viewer3D(Settings settings, Map &map)
    : window_name(settings.window_name),
      window_width(settings.windowWidth),
      window_height(settings.windowHeight),
      ViewpointX(settings.ViewpointX),
      ViewpointY(settings.ViewpointY),
      ViewpointZ(settings.ViewpointZ),
      ViewpointF(settings.ViewpointF),
      is_running(true),
      map(map)

{
  pangolin::CreateWindowAndBind(window_name, window_width, window_height);
  glEnable(GL_DEPTH_TEST);
  // glEnable(GL_BLEND);
  pangolin::GetBoundWindow()->RemoveCurrent();
}
Viewer3D::~Viewer3D() {}

// void Viewer3D::setup(
//     std::string window_name,
//     int window_width,
//     int window_height,
//     int ViewpointX,
//     int ViewpointY,
//     float ViewpointZ,
//     int ViewpointF)
// {
//   this->window_name = window_name;
//   this->window_width = window_width;
//   this->window_height = window_height;
//   this->ViewpointX = ViewpointX;
//   this->ViewpointY = ViewpointY;
//   this->ViewpointZ = ViewpointZ;
//   this->ViewpointF = ViewpointF;
//   this->is_running = true;

//   pangolin::CreateWindowAndBind(window_name, window_width, window_height);
//   glEnable(GL_DEPTH_TEST);
//   // glEnable(GL_BLEND);
//   pangolin::GetBoundWindow()->RemoveCurrent();
// }

void Viewer3D::run()
{
  pangolin::BindToContext(window_name);
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(
          this->window_width,
          this->window_height,
          this->ViewpointF,
          this->ViewpointF,
          this->window_width / 2,
          this->window_height / 2,
          0.1,
          1000),
      pangolin::ModelViewLookAt(
          this->ViewpointX, this->ViewpointY, this->ViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

  // Define Projection and initial ModelView matrix
  // ProjectionMatrix arguemnts
  // widht, height, ?, ?, widht//2, height//2, Near Clipping Plane, Far
  // Clipping Plane

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(
              0.0,
              1.0,
              0.0,
              1.0,
              -static_cast<double>(this->window_width) / static_cast<double>(this->window_height))
          .SetHandler(&handler);

  glPointSize(3);
  Eigen::Matrix3d K;
  K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;
  Eigen::Matrix3d Kinv = K.inverse();
  while (!pangolin::ShouldQuit() && this->is_running)
  {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    pangolin::glDrawAxis(1);
    // pangolin::glDrawFrustum(Kinv, 1241, 376, 1.0);

    for (const auto &[id, keyFrame] : this->map.getKeyFrames())
    {
      // glColor3f(key_frame.color.red, key_frame.color.green,
      // key_frame.color.blue); glColor3f(1., 1., 1.);
      pangolin::glDrawFrustum(Kinv, 1241, 376, keyFrame->getTwcEigen(), 1.0);
      s_cam.Follow(keyFrame->getTwcEigen(), true);
    }

    glBegin(GL_POINTS);
    for (const auto &[id, mapPoint] : this->map.getMapPoints())
    {
      // glColor3f(map_point.color.red, map_point.color.green,
      // map_point.color.blue);
      PosD pos = mapPoint->getPos();
      glVertex3d(pos.x, pos.y, pos.z);
      // glVertex3f(1.0,1.0,1.0);
      // PRINT(map_point.x, map_point.x, map_point.z);
    }
    glVertex3f(1.0, 1.0, 1.0);
    glEnd();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
  pangolin::GetBoundWindow()->RemoveCurrent();
}

std::thread Viewer3D::runWithThread(Viewer3D &viewer3d)
{
  std::thread viewer_thread([&viewer3d] { viewer3d.run(); });
  return viewer_thread;
}

void Viewer3D::stop_thread() { this->is_running = false; }

// void Viewer3D::set_key_frames(const std::vector<Key_frame_dao> &key_frames)
// {
//   // std::unique_lock<std::mutex> lock(this->key_frames_mutex);
//   std::scoped_lock<std::mutex> lock(this->key_frames_mutex);
//   this->key_frames = key_frames;
// }

// void Viewer3D::set_map_points(const std::vector<Map_point_dao> &map_points)
// {
//   // std::unique_lock<std::mutex> lock(this->map_points_mutex);
//   std::scoped_lock<std::mutex> lock(this->map_points_mutex);
//   this->map_points = map_points;
// }

// void Viewer3D::clear_data()
// {
//   std::scoped_lock<std::mutex, std::mutex> lock(
//       this->key_frames_mutex, this->map_points_mutex);
//   this->key_frames.clear();
//   this->map_points.clear();
// }