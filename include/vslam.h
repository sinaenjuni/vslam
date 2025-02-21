#include "camera.h"
#include "feature.h"
#include "misc.h"
#include "settings.h"
#include "viewer3d.h"

class VSLAM
{
 private:
  Settings settings;
  Camera camera;
  IFeature_extractor *feature_extractor;
  IFeature_matcher *feature_matcher;

 public:
  VSLAM();
  ~VSLAM();
  void run();
};