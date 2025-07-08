#include "laser_scanner_utils.h"
#include <climits>

ScanPair LaserScannerUtils::get_min_range(const sensor_msgs::LaserScan& msg) {
  double min = DBL_MAX;
  int index = -1;
  for (int i = 0; i < msg.ranges.size(); i++) {
    const auto& val = msg.ranges[i];
    if (!std::isnan(val) && val < min) {
      min = val;
      index = i;
    }
  }

  return {min, index};
}

ScanPair LaserScannerUtils::get_max_range(const sensor_msgs::LaserScan& msg) {
  double max = -DBL_MAX;
  int index = -1;
  for (int i = 0; i < msg.ranges.size(); i++) {
    const auto& val = msg.ranges[i];
    if (!std::isnan(val) && val > max) {
      max = val;
      index = i;
    }
  }

  return {max, index};
}

double LaserScannerUtils::get_avg_range(const sensor_msgs::LaserScan& msg) {
  double sum = 0;
  long long count = 0;

  for (auto& val : msg.ranges) {
    if (!std::isnan(val)) {
      sum += val;
      count += 1;
    }
  }

  return sum / count;
}

bool LaserScannerUtils::is_obstacle_too_close(const sensor_msgs::LaserScan& msg, int start_index, int end_index, double distance_threshold) {
  double min = DBL_MAX;
  for (int i = start_index; i < end_index; i++) {
    double val = msg.ranges[i];
    if (!std::isnan(val) && val < min) {
      min = val;
    }
  }

  return min < distance_threshold;
}
