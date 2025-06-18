#include <ros/ros.h>
#include <random>
#include <ctime>

// Include custom messages
#include "custom_msgs/IoTSensor.h"

void log_data(const custom_msgs::IoTSensor& data_point) {
  ROS_INFO("id: %d\nname: %s\ntemperature: %lf\nhumidity: %lf\n",
         data_point.id,
         data_point.name.c_str(),
         data_point.temperature,
         data_point.humidity);
}

int random_int(int max) {
  return std::rand() % max;
}

// Define node actions
int main(int argc, char* argv[]) {
  std::srand(std::time(NULL));

  // Initialize node
  ros::init(argc, argv, "cm_sensor");

  // Create a publisher
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<custom_msgs::IoTSensor>("cm_data", 10);

  // Keep publishing new data
  ros::Rate rate(1);
  int i = 0;
  while (ros::ok()) {
    // Create random data point
    custom_msgs::IoTSensor data;
    data.id = i;
    data.name = "IoT Sensor data";
    data.temperature = random_int(255);
    data.humidity = random_int(100);

    // Publish it
    log_data(data);
    pub.publish(data);
    ros::spinOnce();
    rate.sleep();
    ++i;
  }

  return 0;
}
