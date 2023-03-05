/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "serial_bus/implementation.hpp"

#include "serial/factory.hpp"
#include "serial/utils.hpp"

using namespace std::chrono_literals;

namespace serial_bus {

Implementation::Implementation(rclcpp::Node *node)
    : Interface(node) {
  auto prefix = get_prefix_();

  stats_requests_ = node->create_publisher<std_msgs::msg::UInt32>(
      prefix + SERIAL_BUS_TOPIC_REQUESTS, 10);
  stats_responses_succeeded_ = node->create_publisher<std_msgs::msg::UInt32>(
      prefix + SERIAL_BUS_TOPIC_RESPONSES_SUCEEDED, 10);
  stats_responses_failed_ = node->create_publisher<std_msgs::msg::UInt32>(
      prefix + SERIAL_BUS_TOPIC_RESPONSES_FAILED, 10);

  srv_query_ =
      node_->create_service<serial_bus::srv::Query>(
          prefix + SERIAL_BUS_SERVICE_QUERY,
          std::bind(&Implementation::query_handler_, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);
  prov_ = serial::Factory::New(node);
  prov_->register_input_cb(&Implementation::input_cb_, this);
}

/* static */ void Implementation::input_cb_(const std::string &msg,
                                            void *user_data) {
  (void)msg;
  (void)user_data;

  Implementation *that = (Implementation *)user_data;
  that->input_cb_real_(msg);
}

void Implementation::input_cb_real_(const std::string &msg) {
  input_promises_mutex_.lock();

  // TODO(clairbee): optimize it to avoid excessive copying
  input_queue_ += msg;
  RCLCPP_DEBUG(node_->get_logger(), "Received data: %s",
               (serial::utils::bin2hex(msg)).c_str());
  RCLCPP_DEBUG(node_->get_logger(), "Queued data: %s",
               (serial::utils::bin2hex(input_queue_)).c_str());

  if (input_promises_.size() < 1) {
    // No one is waiting for anything.
    RCLCPP_DEBUG(node_->get_logger(), "Discarded unwanted data: %s",
               (serial::utils::bin2hex(msg)).c_str());
    input_promises_mutex_.unlock();
    return;
  }
  
  auto first_promise = input_promises_.begin();
  auto expected_len = first_promise->expected_response_len;
  if (input_queue_.length() < expected_len) {
    // there is not enough data yet
    input_promises_mutex_.unlock();
    SERIAL_BUS_PUBLISH_INC(UInt32, stats_responses_failed_, 1);
    return;
  }

  std::string response(&input_queue_[0], expected_len);
  first_promise->promise.set_value(response);
  input_queue_.erase(0, expected_len);
  input_promises_.erase(first_promise);

  // If there is anyone else in line
  // then send out their request
  if (input_promises_.size() > 0) {
    first_promise = input_promises_.begin();
    prov_->output(first_promise->output);
  }

  input_promises_mutex_.unlock();
}

std::string Implementation::send_request_(uint8_t expected_response_len,
                                          const std::string &output) {
  bool do_send = false;

  input_promises_mutex_.lock();
  if (input_promises_.size() == 0) {
    do_send = true;
  }
  input_promises_.emplace_back(expected_response_len, output);
  std::future<std::string> f = input_promises_.back().promise.get_future();
  input_promises_mutex_.unlock();


  // Make sure to write the request if the promise is the first one in line
  if (do_send) {
    prov_->output(output);
  }

  SERIAL_BUS_PUBLISH_INC(UInt32, stats_requests_, 1);

  // TODO(clairbee): implement a timeout here
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(), "Future arrived");

  std::string result = f.get();
  RCLCPP_DEBUG(node_->get_logger(), "Received serial bus response: %s",
               (serial::utils::bin2hex(result)).c_str());

  return result;
}

std::string Implementation::query(uint8_t expected_response_len,
                                  const std::string &request) {
  auto req = std::make_shared<serial_bus::srv::Query::Request>();
  auto resp = std::make_shared<serial_bus::srv::Query::Response>();

  req->expected_response_len = expected_response_len;
  req->request = std::vector<uint8_t>(request.begin(), request.end());

  query_handler_(req, resp);

  return std::string(resp->response.begin(), resp->response.end());
}

rclcpp::FutureReturnCode Implementation::query_handler_(
    const std::shared_ptr<serial_bus::srv::Query::Request> request,
    std::shared_ptr<serial_bus::srv::Query::Response> response) {
  auto result = send_request_(
    request->expected_response_len,
    std::string(request->request.begin(), request->request.end()));
  if (result.length() != request->expected_response_len) {
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }
  response->response = std::vector<uint8_t>(result.begin(), result.end());
  return rclcpp::FutureReturnCode::SUCCESS;
}

}  // namespace serial_bus
