#include <ros/ros.h>

// Include service definitions
#include "services/AddTwoInts.h"

// Callback for handling the service
bool handle_add_two_ints(services::AddTwoInts::Request& req, services::AddTwoInts::Response& res) {
  res.sum = req.a + req.b;
  return true;
}

// Define node actions
int main(int argc, char* argv[]) {
  // Initialize the node
  ros::init(argc, argv, "se_server");

  // Create the service
  // - NodeHandle node
  // - Name of the service
  // - Callback function to call when request arrives
  ros::NodeHandle node;
  ros::ServiceServer service = node.advertiseService("add_two_ints", handle_add_two_ints);
  std::cout << "Started service /add_two_ints\n";

  // Start listening for requests
  ros::spin();

  return 0;
}
