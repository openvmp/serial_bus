/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "serial_bus/interface_remote.hpp"

#include <functional>

namespace serial_bus {

RemoteInterface::RemoteInterface(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

  RCLCPP_DEBUG(node_->get_logger(),
               "serial_bus::RemoteInterface::RemoteInterface(): Connecting to the "
               "remote interface: %s",
               prefix.c_str());

  clnt_query =
      node->create_client<serial_bus::srv::Query>(
          prefix + SERIAL_BUS_SERVICE_QUERY,
          ::rmw_qos_profile_default, callback_group_);

  clnt_query->wait_for_service();

  RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
               prefix.c_str());
}

std::string RemoteInterface::query(uint8_t expected_response_len,
                                   const std::string &request) {
  auto req = std::make_shared<serial_bus::srv::Query::Request>();
  auto resp = std::make_shared<serial_bus::srv::Query::Response>();

  req->expected_response_len = expected_response_len;
  req->request = std::vector<uint8_t>(request.begin(), request.end());

  auto f = clnt_query->async_send_request(req);
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteInterface::holding_register_read(): response received");
  *resp = *f.get();

  return std::string(resp->response.begin(), resp->response.end());
}

}  // namespace serial_bus
