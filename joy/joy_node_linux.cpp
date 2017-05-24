// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the <ORGANIZATION> nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros-drivers/joystick_drivers/blob/2179759/joy/src/joy_node.cpp

#include <fcntl.h>
#include <linux/joystick.h>
#include <math.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <cstdio>
#include <cstdlib>
#include <memory>

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
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("joy_node");

  rmw_qos_profile_t joy_qos_profile;
  joy_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  joy_qos_profile.depth = 10;
  joy_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  joy_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  auto pub = node->create_publisher<sensor_msgs::msg::Joy>(
    "joy", joy_qos_profile);

  // configurable values
  double deadzone = 0.05;
  double coalesce_interval = 0.001;
  double autorepeat_rate = 20;

  // derived values
  double scale = -1.0 / (1.0 - deadzone) / 32767.0;
  double unscaled_deadzone = 32767.0 * deadzone;
  double autorepeat_interval = 1 / autorepeat_rate;

  js_event event;
  struct timeval tv;
  fd_set set;
  int joy_fd;

  // Big while loop opens, publishes
  while (rclcpp::ok()) {
    bool first_fault = true;
    while (true) {
      rclcpp::spin_some(node);
      joy_fd = open(g_joy_dev_path, O_RDONLY);
      if (joy_fd != -1) {
        // There seems to be a bug in the driver or something where the
        // initial events that are to define the initial state of the
        // joystick are not the values of the joystick when it was opened
        // but rather the values of the joystick when it was last closed.
        // Opening then closing and opening again is a hack to get more
        // accurate initial state data.
        close(joy_fd);
        joy_fd = open(g_joy_dev_path, O_RDONLY);
      }
      if (joy_fd != -1) {
        break;
      }
      if (first_fault) {
        fprintf(stdout, "Couldn't open joystick %s. Will retry every second.\n", g_joy_dev_path);
        first_fault = false;
      }
      sleep(1.0);
    }

    fprintf(stdout, "Opened joystick %s. deadzone: %f.\n", g_joy_dev_path, deadzone);

    tv.tv_sec = 1;
    tv.tv_usec = 0;

    auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
    joy_msg->header.stamp.sec = 0;
    joy_msg->header.stamp.nanosec = 0;
    joy_msg->header.frame_id = "joy";
    joy_msg->axes.resize(2);
    joy_msg->axes[0] = joy_msg->axes[1] = 0;
    joy_msg->buttons.resize(1);
    joy_msg->buttons[0] = 0;

    bool tv_set = false;
    bool publication_pending = false;

    while (rclcpp::ok()) {
      rclcpp::spin_some(node);

      bool publish_now = false;
      bool publish_soon = false;

      FD_ZERO(&set);
      FD_SET(joy_fd, &set);

      int select_out = select(joy_fd + 1, &set, NULL, NULL, &tv);
      if (select_out == -1) {
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        continue;
      }

      if (FD_ISSET(joy_fd, &set)) {
        if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN) {
          break;  // Joystick is probably closed. Definitely occurs.
        }

        joy_msg->header.stamp = rclcpp::Time::now();
        switch (event.type) {
          case JS_EVENT_BUTTON:
          case JS_EVENT_BUTTON | JS_EVENT_INIT:
            if (event.number >= joy_msg->buttons.size()) {
              int old_size = joy_msg->buttons.size();
              joy_msg->buttons.resize(event.number + 1);
              for (unsigned int i = old_size; i < joy_msg->buttons.size(); ++i) {
                joy_msg->buttons[i] = 0.0;
              }
            }
            joy_msg->buttons[event.number] = (event.value ? 1 : 0);
            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            if (!(event.type & JS_EVENT_INIT)) {
              publish_now = true;
            } else {
              publish_soon = true;
            }
            break;
          case JS_EVENT_AXIS:
          case JS_EVENT_AXIS | JS_EVENT_INIT:
            if (event.number >= joy_msg->axes.size()) {
              int old_size = joy_msg->axes.size();
              joy_msg->axes.resize(event.number + 1);
              for (unsigned int i = old_size; i < joy_msg->axes.size(); i++) {
                joy_msg->axes[i] = 0.0;
              }
            }
            if (!(event.type & JS_EVENT_INIT)) {  // Init event.value is wrong.
              double val = event.value;
              // Allows deadzone to be "smooth"
              if (val > unscaled_deadzone) {
                val -= unscaled_deadzone;
              } else if (val < -unscaled_deadzone) {
                val += unscaled_deadzone;
              } else {
                val = 0;
              }
              joy_msg->axes[event.number] = val * scale;
            }
            // Will wait a bit before sending to try to combine events.
            publish_soon = true;
            break;
          default:
            fprintf(stderr,
              "joy_node: Unknown event type. Please file a ticket. "
              "time=%u, value=%d, type=%Xh, number=%d",
              event.time, event.value, event.type, event.number);
            break;
        }
      } else if (tv_set) {
        publish_now = true;
      }

      if (publish_now) {
        // Assume that all the JS_EVENT_INIT messages have arrived already.
        // This should be the case as the kernel sends them along as soon as
        // the device opens.
        pub->publish(joy_msg);
        publish_now = false;
        tv_set = false;
        publication_pending = false;
        publish_soon = false;
      }

      // If an axis event occurred, start a timer to combine with other
      // events.
      if (!publication_pending && publish_soon) {
        tv.tv_sec = trunc(coalesce_interval);
        tv.tv_usec = (coalesce_interval - tv.tv_sec) * 1e6;
        publication_pending = true;
        tv_set = true;
      }

      // If nothing is going on, start a timer to do autorepeat.
      if (!tv_set && autorepeat_rate > 0) {
        tv.tv_sec = trunc(autorepeat_interval);
        tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
        tv_set = true;
      }

      if (!tv_set) {
        tv.tv_sec = 1;
        tv.tv_usec = 0;
      }
    }

    close(joy_fd);
    rclcpp::spin_some(node);
    if (rclcpp::ok()) {
      fprintf(stdout, "Connection to joystick device lost unexpectedly. Will reopen.");
    }
  }

  return 0;
}
