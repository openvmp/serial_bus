/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_BUS_INTERFACE_H
#define OPENVMP_SERIAL_BUS_INTERFACE_H

#include "ros2_serial_bus/srv/query.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"

#define SERIAL_BUS_SERVICE_QUERY "/query"
#define SERIAL_BUS_TOPIC_REQUESTS "/requests"
#define SERIAL_BUS_TOPIC_RESPONSES_SUCEEDED "/responses_suceeded"
#define SERIAL_BUS_TOPIC_RESPONSES_FAILED "/responses_failed"

namespace ros2_serial_bus {

class Interface {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface() {}

  virtual std::string query(uint8_t expected_response_len,
                            const std::string &request) = 0;

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string get_prefix_();

 private:
  rclcpp::Parameter interface_prefix_;
};

}  // namespace ros2_serial_bus

#endif  // OPENVMP_SERIAL_BUS_INTERFACE_H
