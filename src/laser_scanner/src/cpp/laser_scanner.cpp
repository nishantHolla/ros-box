#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "laser_scanner_utils.h"

void callback(const sensor_msgs::LaserScan& msg) {
  ScanPair min_value = LaserScannerUtils::get_min_range(msg);
  ScanPair max_value = LaserScannerUtils::get_max_range(msg);

  std::cout << "minimum range: " << min_value.value << " at index " << min_value.index << "\n";
  std::cout << "maximum range: " << max_value.value << " at index " << max_value.index << "\n";
  std::cout << "average range: " << LaserScannerUtils::get_avg_range(msg) << "\n";

  if (LaserScannerUtils::is_obstacle_too_close(msg, 0, 600, 0.69)) {
    std::cout << "obstacle too close" << "\n";
  }

  std::cout << "\n";
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "laser_scanner");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/scan", 10, callback);

  ros::spin();
  return 0;
}
