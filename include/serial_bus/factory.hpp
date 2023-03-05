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

#include "serial_bus/interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace serial_bus {

class Factory {
 public:
  static std::shared_ptr<Interface> New(rclcpp::Node *node);
};

}  // namespace serial_bus

#endif  // OPENVMP_SERIAL_BUS_FACTORY_H