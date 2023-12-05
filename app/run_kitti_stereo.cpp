//
// Created by gaoxiang on 19-5-4.
//

#include "myslam/visual_odometry.h"
#include <gflags/gflags.h>

DEFINE_string(config_file, "/home/cw/Slam/VSLAM/config/default.yaml",
              "config file path");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
  assert(vo->Init() == true);
  vo->Run();

  return 0;
}
