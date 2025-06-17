#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"

// Actions that can be performed by the agent
const std::vector<std::string> ACTIONS {
  "TURN_LEFT",
  "TURN_RIGHT",
  "FORWARD"
};

// Definition of PI for turning left and right
constexpr double PI = M_PI;

// Get the next action to perform from the user
int get_next_action(bool& to_quit) {
  std::string input;
  int selection;

  // Keep asking until valid action is entered
  while (true) {
    for (int i = 0, e = ACTIONS.size(); i < e; ++i) {
      std::cout << i << ": " << ACTIONS[i] << "\n";
    }
    std::cout << "> ";
    std::cin >> input;

    if (input == "q") {
      to_quit = true;
      return 0;
    }

    try {
      int val = std::stoi(input);
      if (val >= 0 && val < ACTIONS.size()) {
        return val;
      }
    }
    catch (...) {}
  }
}

// Perform the given action by pulishing new Twist message using the publisher
void perform_action(const std::string& action, ros::Publisher& publisher) {
  const double LINEAR_SPEED = 3;       // 3 units per sec
  const double ANGULAR_SPEED = PI / 2; // 90 degs per sec

  ros::Rate rate(1);

  // Create the twist object
  geometry_msgs::Twist command;

  // Update the message parameters depending on the action
  if (action == "TURN_LEFT") {
    command.angular.z = ANGULAR_SPEED; // Positive is anticlockwise
  }
  else if (action == "TURN_RIGHT") {
    command.angular.z = -ANGULAR_SPEED; // Negative is clockwise
  }
  else if (action == "FORWARD") {
    command.linear.x = LINEAR_SPEED;
  }

  // Publish the message
  publisher.publish(command);

  // Do the action for 1 second and then reset
  rate.sleep();
}

// Define actions of the node
int main(int argc, char* argv[]) {
  // Initialize the node
  ros::init(argc, argv, "turtle_agent");

  // Create a publisher to /turtle1/cmd_vel
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  // Node loop
  while (ros::ok()) {
    bool to_quit = false;
    int action_idx = get_next_action(to_quit);

    // If to quit, stop
    if (to_quit) {
      break;
    }

    perform_action(ACTIONS[action_idx], pub);
  }

  return 0;
}
