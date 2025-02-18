#include "vslam.h"
#include "settings.h"

VSLAM::VSLAM() {
  Settings settings = prase_settings("../settings/KITTI_Monocular.yaml");
  settings.print();
}

VSLAM::~VSLAM() {}

void VSLAM::run() {}
