#ifndef LASER_SCANNER_UTILS_H_
#define LASER_SCANNER_UTILS_H_

#include "sensor_msgs/LaserScan.h"

struct ScanPair {
  double value;
  int index;
};

class LaserScannerUtils {
public:
  static ScanPair get_min_range(const sensor_msgs::LaserScan& msg);
  static ScanPair get_max_range(const sensor_msgs::LaserScan& msg);
  static double get_avg_range(const sensor_msgs::LaserScan& msg);
  static bool is_obstacle_too_close(const sensor_msgs::LaserScan& msg, int start_index, int end_index, double distance_threshold);
};

#endif // !LASER_SCANNER_UTILS_H_
