// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"


// the equivalent for OS X seems to be IOKit/hid/IOHIDLib.h
// but it's completely different in basically every way possible
// there will also be a completely different Windows-specific driver
// these should probably get their own source files; refactor at some point

/**
 * at some point, we will want to take in the device name though
 * the 'dev' parameter as in ROS 1. For now, we'll assume /dev/input/js0
 */
static const char * g_joy_dev_path = "/dev/input/js0";

int main(int argc, char * argv[])
{
  int joy_fd = open(g_joy_dev_path, O_RDONLY);
  if (joy_fd < 0) {
    printf("ahhhh couldn't open %s\n", g_joy_dev_path);
    return 1;
  } else {
    printf("opened %s as fd %d\n", g_joy_dev_path, joy_fd);
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("joy_node");
  auto joy_pub = node->create_publisher<sensor_msgs::msg::Joy>(
    "joy", rmw_qos_profile_sensor_data);

  auto msg = std::make_shared<sensor_msgs::msg::Joy>();
  msg->header.stamp.sec = 0;
  msg->header.stamp.nanosec = 0;
  msg->header.frame_id = "joy";
  msg->axes.resize(2);
  msg->axes[0] = msg->axes[1] = 0;
  msg->buttons.resize(1);
  msg->buttons[0] = 0;

  fd_set read_fds;
  struct timeval tv;
  int loop_count = 0;

  while (rclcpp::ok()) {
    loop_count++;
    bool something_happened = false;
    for (int num_events = 0; num_events < 10; num_events++) {
      // we want to drain the joy device before publishing, but not
      // spend too much time in here in case the axis is moving a lot
      FD_ZERO(&read_fds);
      FD_SET(joy_fd, &read_fds);
      tv.tv_sec = 0;
      tv.tv_usec = 5000;  // wait at most 5ms. this could be a lot smarter.
      int retval = select(joy_fd + 1, &read_fds, NULL, NULL, &tv);
      if (retval == -1) {
        perror("select()");
      } else if (retval > 0) {
        struct js_event e;
        int nread = read(joy_fd, &e, sizeof(e));
        if (nread != sizeof(struct js_event)) {
          printf("ahhhhh read %d bytes instead of %d\n",
            nread, static_cast<int>(sizeof(struct js_event)));
          break;
        }
        switch (e.type & 0x7f) {
          case JS_EVENT_BUTTON:
            if (e.number >= msg->buttons.size()) {
              msg->buttons.resize(e.number + 1);
            }
            msg->buttons[e.number] = e.value;
            something_happened = true;
            break;
          case JS_EVENT_AXIS:
            if (e.number >= msg->axes.size()) {
              msg->axes.resize(e.number + 1);
            }
            msg->axes[e.number] = e.value / 32767.0;
            something_happened = true;
            break;
          default:
            break;
        }
      } else {
        break;  // if we timed out, we've drained the device; now send ros msg.
      }
    }
    if (something_happened) {
      joy_pub->publish(msg);
    }
    // avoid calling spin_some too often, since it's a bit heavy right now.
    if (something_happened || loop_count % 100 == 0) {
      rclcpp::spin_some(node);
    }
  }
  return 0;
}
