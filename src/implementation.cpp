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

#ifndef DEBUG
#undef RCLCPP_DEBUG
#define RCLCPP_DEBUG(...)
#endif

namespace serial_bus {

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

  stats_requests_ = node->create_publisher<std_msgs::msg::UInt32>(
      prefix + SERIAL_BUS_TOPIC_REQUESTS, qos);
  stats_responses_succeeded_ = node->create_publisher<std_msgs::msg::UInt32>(
      prefix + SERIAL_BUS_TOPIC_RESPONSES_SUCEEDED, qos);
  stats_responses_failed_ = node->create_publisher<std_msgs::msg::UInt32>(
      prefix + SERIAL_BUS_TOPIC_RESPONSES_FAILED, qos);

  srv_query_ = node_->create_service<serial_bus::srv::Query>(
      prefix + SERIAL_BUS_SERVICE_QUERY,
      std::bind(&Implementation::query_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
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
  RCLCPP_DEBUG(node_->get_logger(), "Received data: %s",
               (serial::utils::bin2hex(msg)).c_str());

  input_promises_mutex_.lock();
  input_queue_ += msg;  // TODO(clairbee): optimize it to reduce extra copying
  RCLCPP_DEBUG(node_->get_logger(), "Queued data: %s",
               (serial::utils::bin2hex(input_queue_)).c_str());

  do {
    if (input_promises_.size() < 1) {
      // No one is waiting for anything.
      break;
    }

    // See how much data the requestor wanted
    auto first_promise = input_promises_.begin();
    auto expected_len = (*first_promise)->expected_response_len;
    if (input_queue_.length() < expected_len) {
      // there is not enough data yet
      SERIAL_BUS_PUBLISH_INC(UInt32, stats_responses_failed_, 1);
      input_promises_mutex_.unlock();
      return;
    }

    // Extract just the right amount of data
    std::string response(&input_queue_[0], expected_len);
    (*first_promise)->promise.set_value(response);
    input_queue_.erase(0, expected_len);

    // Remove the requestor from the queue
    input_promises_.erase(first_promise);
    SERIAL_BUS_PUBLISH_INC(UInt32, stats_responses_succeeded_, 1);

    // If there is anyone else waiting in line
    // then send out their request
    if (input_promises_.size() > 0) {
      first_promise = input_promises_.begin();
      prov_->output((*first_promise)->output);
    }
  } while (false);

  // Whatever is left is something no one asked for
  if (input_queue_.length() > 0) {
    RCLCPP_DEBUG(node_->get_logger(), "Discarding unwanted data: %s",
                 (serial::utils::bin2hex(input_queue_)).c_str());
    input_queue_ = "";
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
  auto input_promise = std::make_shared<Promise>(expected_response_len, output);
  input_promise->start = std::chrono::steady_clock::now();
  input_promises_.push_back(input_promise);
  std::future<std::string> f = input_promise->promise.get_future();
  input_promises_mutex_.unlock();

  // Make sure to write the request if the promise is the first one in line
  if (do_send) {
    prov_->output(output);
  }

  SERIAL_BUS_PUBLISH_INC(UInt32, stats_requests_, 1);

#define INPUT_QUEUE_TIMEOUT_MS 17
  auto status = f.wait_for(std::chrono::milliseconds(INPUT_QUEUE_TIMEOUT_MS));
  if (status == std::future_status::timeout) {
    RCLCPP_ERROR(node_->get_logger(),
                 "serial_bus: send_request_(): future timed out");
    input_promises_mutex_.lock();
    auto p = input_promises_.begin();
    for (; p != input_promises_.end(); p++) {
      if ((*p) == input_promise) {
        // remove ourselves from the list of waiters
        input_promises_.erase(p);
        break;
      }
    }
    input_promises_mutex_.unlock();
    return "";
  }
  RCLCPP_DEBUG(node_->get_logger(), "Future arrived");

  std::string result = f.get();

#ifdef DEBUG
  auto end = std::chrono::steady_clock::now();
#endif
  RCLCPP_DEBUG(node_->get_logger(), "Received serial bus response: %s in %dms",
               (serial::utils::bin2hex(result)).c_str(),
               (int)(std::chrono::duration_cast<std::chrono::milliseconds>(
                         end - input_promise->start)
                         .count()));

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
