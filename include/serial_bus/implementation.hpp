/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_BUS_IMPLEMENTATION_H
#define OPENVMP_SERIAL_BUS_IMPLEMENTATION_H

#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/interface.hpp"
#include "serial_bus/interface.hpp"
#include "std_msgs/msg/u_int32.hpp"

#define SERIAL_BUS_PUBLISH(type, name, value) \
  {                                           \
    std_msgs::msg::type msg;                  \
    msg.data = value;                         \
    name->publish(msg);                       \
  }
#define SERIAL_BUS_PUBLISH_INC(type, name, inc)   \
  {                                               \
    name##_value_ += inc;                         \
    SERIAL_BUS_PUBLISH(type, name, name##_value_) \
  }

namespace serial_bus {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node);
  virtual ~Implementation() {}

 protected:
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr stats_requests_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr
      stats_responses_succeeded_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr stats_responses_failed_;

  virtual std::string query(uint8_t expected_response_len,
                            const std::string &request) override;

 private:
  std::shared_ptr<serial::Interface> prov_;

  class Promise {
   public:
    Promise(uint8_t expected_response_len, const std::string &output)
        : expected_response_len{expected_response_len}, output{output} {}
    uint8_t expected_response_len;
    std::string output;
    std::promise<std::string> promise;
    std::chrono::steady_clock::time_point start;
  };
  // input_promises_ contain everyone waiting for their lef_id to repspond
  std::vector<std::shared_ptr<Promise>> input_promises_;
  // input_queue_ accumulates data from previous read()s until the entire frame
  // arrives
  std::string input_queue_;
  std::mutex input_promises_mutex_;

  // published values
  uint32_t stats_requests__value_;
  uint32_t stats_responses_succeeded__value_;
  uint32_t stats_responses_failed__value_;

  // input_cb is a static method to receive callbacks from the serial module.
  static void input_cb_(const std::string &msg, void *user_data);
  // input_cb_real_ is the real handler of the callbacks from the serial module.
  void input_cb_real_(const std::string &msg);
  // send_request_ send a request and waits for a response
  std::string send_request_(uint8_t expected_response_len,
                            const std::string &output);

  rclcpp::Service<serial_bus::srv::Query>::SharedPtr srv_query_;
  rclcpp::FutureReturnCode query_handler_(
      const std::shared_ptr<serial_bus::srv::Query::Request> request,
      std::shared_ptr<serial_bus::srv::Query::Response> response);
};

}  // namespace serial_bus

#endif  // OPENVMP_SERIAL_BUS_IMPLEMENTATION_H
