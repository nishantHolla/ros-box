#include <ros/ros.h>
#include <sstream>

// Importing String message from std_msgs package
#include "std_msgs/String.h"

// Define actions of the node
int main(int argc, char* argv[]) {
  // Initialize the node
  // - argc and argv
  // - Name of the node
  ros::init(argc, argv, "ps_talker");

  // Create a publisher
  // - NodeHandle node
  // - Type of the topic as template argument
  // - Name of the topic
  // - Buffer size to hold messages
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<std_msgs::String>("/my_message", 10);

  // Rate of publication
  ros::Rate rate(1); // 1 Hz

  // Start publishing messages
  int i = 0;
  while (ros::ok()) {
    std_msgs::String message;               // Create the message
    std::stringstream ss;
    ss << "Hello world " << i;
    message.data = ss.str();
    ROS_INFO("%s", message.data.c_str());   // Log the message to stdout
    pub.publish(message);                   // Publish the message
    ros::spinOnce();                        // Wait for the message to be published
    rate.sleep();                           // Sleep for a while so the desired rate of sending is met
    ++i;                                    // Increment the counter
  }

  return 0;
}
