/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_serial_bus/interface_remote.hpp"

#include <functional>

namespace remote_serial_bus {

RemoteInterface::RemoteInterface(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

  RCLCPP_DEBUG(
      node_->get_logger(),
      "serial_bus::RemoteInterface::RemoteInterface(): Connecting to the "
      "remote interface: %s",
      prefix.c_str());

  clnt_query = node->create_client<srv::Query>(
      prefix + SERIAL_BUS_SERVICE_QUERY, ::rmw_qos_profile_default,
      callback_group_);

  clnt_query->wait_for_service();

  RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
               prefix.c_str());
}

std::string RemoteInterface::query(uint8_t expected_response_len,
                                   const std::string &request) {
  auto req = std::make_shared<srv::Query::Request>();

  req->expected_response_len = expected_response_len;
  req->request = std::vector<uint8_t>(request.begin(), request.end());

  auto f = clnt_query->async_send_request(req);
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteInterface::query(): response received");
  auto resp = f.get();

  return std::string(resp->response.begin(), resp->response.end());
}

}  // namespace remote_serial_bus
