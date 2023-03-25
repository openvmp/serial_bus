/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_serial_bus/node.hpp"

namespace ros2_serial_bus {

Node::Node() : rclcpp::Node::Node("serial_bus") {
  impl_ = std::make_shared<Implementation>(this);
}

}  // namespace ros2_serial_bus
