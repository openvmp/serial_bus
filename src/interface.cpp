/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_serial_bus/interface.hpp"

#include <functional>

#include "ros2_serial_bus/srv/query.hpp"


namespace ros2_serial_bus {

Interface::Interface(rclcpp::Node *node) : node_{node} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("serial_bus_prefix", "/serial_bus/" + std::string(node_->get_name()));
  node->get_parameter("serial_bus_prefix", interface_prefix_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() >0 && prefix[prefix.length()-1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}


}  // namespace ros2_serial_bus
