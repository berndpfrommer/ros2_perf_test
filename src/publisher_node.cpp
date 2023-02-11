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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using StringMsg = std_msgs::msg::String;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "publisher_node [-r rate] [-s msg_size] [-q q_size]"
            << std::endl;
}

struct TestPublisher : public rclcpp::Node
{
  explicit TestPublisher(
    const rclcpp::NodeOptions & options, int q_size, int msg_size, double rate)
  : Node("test_publisher", options)
  {
    size_ = msg_size;
    msg_.data.resize(size_, ' ');
    rclcpp::QoS qos(q_size);
    pub_ = create_publisher<StringMsg>("test_string", qos);
    lastTime_ = get_clock()->now();
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      std::bind(&TestPublisher::timerExpired, this));
  }

private:
  void timerExpired()
  {
    pub_->publish(msg_);
    numPub_++;
    const auto t = get_clock()->now();
    if (t - lastTime_ > rclcpp::Duration::from_seconds(1.0)) {
      const double dt = rclcpp::Duration(t - lastTime_).nanoseconds() * 1e-9;
      const double rate = numPub_ / dt;
      const double bw = rate * size_ * 8 * 1e-6;
      RCLCPP_INFO(
        get_logger(), "size: %5d rate: %.2f bw: %8.3f Mbits/s", size_, rate,
        bw);
      numPub_ = 0;
      lastTime_ = t;
    }
  }
  // -- variables
  rclcpp::Time lastTime_;
  rclcpp::TimerBase::SharedPtr timer_;
  StringMsg msg_;
  size_t numPub_{0};
  int size_{0};
  typename rclcpp::Publisher<StringMsg>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int opt;
  int q_size{1};
  int msg_size{1000};
  double rate{40.0};
  while ((opt = getopt(argc, argv, "r:s:q:h")) != -1) {
    switch (opt) {
      case 's':
        msg_size = atoi(optarg);
        break;
      case 'q':
        q_size = atoi(optarg);
        break;
      case 'r':
        rate = atof(optarg);
        break;
      case 'h':
        usage();
        return (-1);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  auto node = std::make_shared<TestPublisher>(
    rclcpp::NodeOptions(), q_size, msg_size, rate);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
