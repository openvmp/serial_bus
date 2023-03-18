/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "serial_bus/factory.hpp"

#include "serial_bus/implementation.hpp"
#include "serial_bus/interface_remote.hpp"

namespace serial_bus {

std::shared_ptr<Interface> Factory::New(rclcpp::Node *node) {
  rclcpp::Parameter use_remote;
  if (!node->has_parameter("use_remote")) {
    node->declare_parameter("use_remote", true);
  }
  node->get_parameter("use_remote", use_remote);

  rclcpp::Parameter is_remote;
  node->declare_parameter("serial_bus_is_remote", use_remote.as_bool());
  node->get_parameter("serial_bus_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node);
  } else {
    return std::make_shared<Implementation>(node);
  }
}

}  // namespace serial_bus