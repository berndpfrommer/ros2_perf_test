// -*-c++-*-
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using namespace std::chrono_literals;

using StringMsg = std_msgs::msg::String;

struct TestSubscriber : public rclcpp::Node
{
  explicit TestSubscriber(const rclcpp::NodeOptions & options)
  : Node("test_subscriber", options)
  {
    lastTime_ = get_clock()->now();
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0),
      std::bind(&TestSubscriber::timerExpired, this));
    rclcpp::QoS qos(1);
    sub_ = create_subscription<StringMsg>(
      "test_string", qos,
      std::bind(&TestSubscriber::callback, this, std::placeholders::_1));
  }

private:
  void timerExpired()
  {
    const auto t = get_clock()->now();
    const double dt = rclcpp::Duration(t - lastTime_).nanoseconds() * 1e-9;
    const double rate = numReceived_ / dt;
    RCLCPP_INFO_STREAM(
      get_logger(), "received " << numReceived_ << " rate: " << rate);
    numReceived_ = 0;
    lastTime_ = t;
  }

  void callback(StringMsg::ConstSharedPtr msg)
  {
    (void)msg;
    this->numReceived_++;
  }

  // -- variables
  int numReceived_{0};
  rclcpp::Time lastTime_;
  rclcpp::TimerBase::SharedPtr timer_;
  typename rclcpp::Subscription<StringMsg>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestSubscriber>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
