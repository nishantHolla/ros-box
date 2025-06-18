#include <ros/ros.h>
#include <string>

// Include service definitions
#include "services/AddTwoInts.h"

// Get two integers from the user
bool get_input(int& a, int& b) {
  std::string a_str, b_str;

  std::cout << "a: ";
  std::cin >> a_str;

  try {
    a = std::stoi(a_str);
  }
  catch (...) {
    return false;
  }

  std::cout << "b: ";
  std::cin >> b_str;

  try {
    b = std::stoi(b_str);
  }
  catch (...) {
    return false;
  }

  return true;
}

// Define node actions
int main(int argc, char* argv[]) {
  // Initialize the node
  ros::init(argc, argv, "se_client");

  // Create the client
  // - NodeHandle node
  // - Type of the service
  // - Name of the service
  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<services::AddTwoInts>("add_two_ints");

  while (ros::ok()) {
    // Create the request
    services::AddTwoInts srv;
    int a, b;
    if (!get_input(a, b)) {
      break;
    }

    srv.request.a = a;
    srv.request.b = b;

    // Send the request
    if (client.call(srv)) {
      std::cout << "sum: " << (long int)srv.response.sum << "\n";
    }
    else {
      ROS_ERROR("Failed to call the service");
    }
  }

  return 0;
}
