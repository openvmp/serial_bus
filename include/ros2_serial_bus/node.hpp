/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_BUS_RTU_NODE_H
#define OPENVMP_SERIAL_BUS_RTU_NODE_H

#include <memory>
#include <string>

#include "ros2_serial_bus/implementation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_serial_bus {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  std::shared_ptr<Implementation> impl_;
};

}  // namespace ros2_serial_bus

#endif  // OPENVMP_SERIAL_BUS_RTU_NODE_H
