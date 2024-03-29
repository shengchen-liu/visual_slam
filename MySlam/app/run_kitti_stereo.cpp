#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "./config/default.yaml", "config file path");

// resolve the error when using CLion to build.
// error: “ERROR: unknown command line flag ‘gtest_color’”,
// “ERROR: unknown command line flag ‘gtest_filter’”
DEFINE_string(gtest_filter, "", "");
DEFINE_string(gtest_color, "", "");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
