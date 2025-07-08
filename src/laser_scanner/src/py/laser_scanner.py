#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import LaserScan
import math

def scan_callback(scan_data):
	min_value, min_index = min_range_index(scan_data.ranges)
	print(f"Minimum range value: {min_value}")
	print(f"Minimum range index: {min_index}")

	max_value, max_index = max_range_index(scan_data.ranges)
	print(f"Maximum range value: {max_value}")
	print(f"Maximum range index: {max_index}")

	avg_value = avg_range(scan_data.ranges)
	print(f"Average range value: {avg_value}")

	avg_btwn_indices = avg_between_indices(scan_data.ranges, 2, 7)
	print(f"Average between 2 and 7: {avg_btwn_indices}")
	print()

def min_range_index(ranges):
	ranges = [x for x in ranges if not math.isnan(x)]
	return (min(ranges), ranges.index(min(ranges)))

def max_range_index(ranges):
	ranges = [x for x in ranges if not math.isnan(x)]
	return (max(ranges), ranges.index(max(ranges)))

def avg_range(ranges):
	ranges = [x for x in ranges if not math.isnan(x)]
	return (sum(ranges) / len(ranges))

def avg_between_indices(ranges, min, max):
	ranges = [x for x in ranges[min:max+1] if not math.isnan(x)]
	return (sum(ranges) / len(ranges))

def node():
	rospy.init_node("laser_scanner", anonymous=False)

	pub = rospy.Subscriber("/scan", LaserScan, scan_callback)

	rospy.spin()


if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass
