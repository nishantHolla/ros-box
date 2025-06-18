#include <ros/ros.h>

#include "rectangle_area_service/RectangleAreaService.h"

bool callback(rectangle_area_service::RectangleAreaService::Request& req,
              rectangle_area_service::RectangleAreaService::Response& res) {
  res.area = req.width * req.height;
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ras_server");

  ros::NodeHandle node;
  ros::ServiceServer server = node.advertiseService("rectangle_area", callback);
  ROS_INFO("Started service /rectangle_area");

  ros::spin();

  return 0;
}
