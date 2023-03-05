/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */
#ifndef OPENVMP_SERIAL_BUS_INTERFACE_REMOTE_H
#define OPENVMP_SERIAL_BUS_INTERFACE_REMOTE_H

#include <future>
#include <memory>
#include <string>

#include "serial_bus/interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace serial_bus {

class RemoteInterface : public Interface {
 public:
  RemoteInterface(rclcpp::Node *node);
  virtual ~RemoteInterface() {}

  virtual std::string query(uint8_t expected_response_len,
                            const std::string &request) override;

 private:
  rclcpp::Client<serial_bus::srv::Query>::SharedPtr clnt_query;
};

}  // namespace modbus

#endif  // OPENVMP_SERIAL_BUS_INTERFACE_REMOTE_H
