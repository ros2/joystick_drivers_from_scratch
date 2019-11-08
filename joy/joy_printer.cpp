// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// so much joy :O :) :]
void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  printf("axes = [ ");
  for (size_t i = 0; i < joy->axes.size(); i++) {
    printf("%+0.3f ", joy->axes[i]);
  }
  printf("] buttons = [ ");
  for (size_t i = 0; i < joy->buttons.size(); i++) {
    printf("%d ", joy->buttons[i]);
  }
  printf(" ]\n");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("joy_printer");
  auto subscription = node->create_subscription<sensor_msgs::msg::Joy>
      ("joy", rclcpp::SensorDataQoS{}, joy_callback);
  rclcpp::spin(node);
  return 0;
}
