#include <ros/ros.h>

// Include custom message
#include "custom_msgs/IoTSensor.h"

void callback(const custom_msgs::IoTSensor& data_point) {
  printf("id: %d\nname: %s\ntemperature: %lf\nhumidity: %lf\n",
         data_point.id,
         data_point.name.c_str(),
         data_point.temperature,
         data_point.humidity);
}

// Define node action
int main(int argc, char* argv[]) {
  // Initialize the node
  ros::init(argc, argv, "cm_screen");

  // Create a subscriber
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("cm_data", 10, callback);

  // Keep listening
  ros::spin();

  return 0;
}
