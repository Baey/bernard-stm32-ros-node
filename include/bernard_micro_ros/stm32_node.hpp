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
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int8.h>

class STM32Node
{
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
  rcl_timer_t timer;        // Fast timer (IMU + feet pressure)
  rcl_timer_t tempTimer;    // Slow timer (temperature)
  rclc_executor_t executor;
  rcl_allocator_t allocator;
  rcl_publisher_t pubFeetPressure;
  rcl_publisher_t pubImu;
  rcl_publisher_t pubTemp;

  // Status fields
  rcl_ret_t retFeetPressure;
  rcl_ret_t retImu;
  rcl_ret_t retTemp;
  rcl_ret_t retTimer;
  rcl_ret_t retTempTimer;
  rcl_ret_t retNode;
  rcl_ret_t retExecutor;

  // Message objects
  std_msgs__msg__UInt16MultiArray msgFeetPressure;
  sensor_msgs__msg__Imu msgImu;
  std_msgs__msg__Int8 msgTemp;

  static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
  static void tempTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif // _STM32_NODE_HPP
