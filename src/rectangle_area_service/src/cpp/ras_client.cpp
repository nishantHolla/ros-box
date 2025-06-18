#include <ros/ros.h>

#include "rectangle_area_service/RectangleAreaService.h"

bool get_input(int& a, int& b) {
  std::string a_str, b_str;

  std::cout << "width: ";
  std::cin >> a_str;

  try {
    a = std::stoi(a_str);
  }
  catch (...) {
    return false;
  }

  std::cout << "height: ";
  std::cin >> b_str;

  try {
    b = std::stoi(b_str);
  }
  catch (...) {
    return false;
  }

  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ras_client");

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<rectangle_area_service::RectangleAreaService>("rectangle_area");

  while (ros::ok()) {
    rectangle_area_service::RectangleAreaService srv;
    int a, b;
    if (!get_input(a, b)) {
      break;
    }

    srv.request.width = a;
    srv.request.height = b;

    if (client.call(srv)) {
      std::cout << "area: " << (long int)srv.response.area << "\n";
    }
    else {
      ROS_ERROR("Failed to call the service");
    }
  }

  return 0;
}
