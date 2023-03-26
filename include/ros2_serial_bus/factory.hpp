/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_BUS_FACTORY_H
#define OPENVMP_SERIAL_BUS_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_serial_bus/interface.hpp"

namespace ros2_serial_bus {

class Factory {
 public:
  static std::shared_ptr<Interface> New(rclcpp::Node *node);
};

}  // namespace ros2_serial_bus

#endif  // OPENVMP_SERIAL_BUS_FACTORY_H
