// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _STM32_NODE_HPP
#define _STM32_NODE_HPP

#include <bernard_micro_ros/macros.hpp>
#include <bernard_sensors.hpp>
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/u_int8.h>

constexpr uint8_t FEET_PRESSURE_ARRAY_BITS_NUM = sizeof(uint16_t) * 2;

class STM32Node {
public:
  STM32Node(BernardSensors &sensors);

  ~STM32Node();

  static STM32Node *instance;

  bool createEntities();

  void destroyEntities();

  void spin();

private:
  BernardSensors *sensors;
  rclc_support_t support;
  rcl_node_t node;
  rcl_timer_t timer;
  rclc_executor_t executor;
  rcl_allocator_t allocator;
  rcl_publisher_t pubFeetPressure;
  rcl_publisher_t pubImu;
  rcl_publisher_t pubStatus;

  // Status fields
  rcl_ret_t retFeetPressure;
  rcl_ret_t retImu;
  rcl_ret_t retStatus;
  rcl_ret_t retTimer;
  rcl_ret_t retNode;
  rcl_ret_t retExecutor;

  // Message objects
  std_msgs__msg__UInt16MultiArray msgFeetPressure;
  sensor_msgs__msg__Imu msgImu;
  std_msgs__msg__UInt8 msgStatus;

  static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif // _STM32_NODE_HPP
