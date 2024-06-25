#include <gflags/gflags.h>
#include "visual_odometry.hpp"
#include "rgbd_vio_node.hpp"

DEFINE_string(config_file_path, "/2024-visual-slam/config/config.yaml", "config file path");

int main(int argc, char **argv) {
//   google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  rclcpp::init(argc, argv);
  auto vio = new  VisualOdometry(FLAGS_config_file_path);
  
  auto node = std::make_shared<RgbdVioNode>(vio);
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  
  return 0;
}