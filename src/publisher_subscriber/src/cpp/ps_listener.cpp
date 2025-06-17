#include <ros/ros.h>

// Importing String message from std_msgs package
#include "std_msgs/String.h"

// Callback function to handle new message
void callback(const std_msgs::String::ConstPtr& msg) {
	// To know the structure of message object, run
	// $ rosmsg info std_msgs/String
  ROS_INFO("%s", msg->data.c_str());
}

// Define actions of the node
int main(int argc, char* argv[]) {
  // Initialize the node
  // - argc and argv
  // - Name of the node
  ros::init(argc, argv, "ps_listener");

  // Create a Subscriber
  // - NodeHandle node
  // - Name of the topic
  // - Buffer size to hold messages
  // - Callback function to execute when a message arrives
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/my_message", 10, callback);

  // Start listening
  ros::spin();

  return 0;
}
