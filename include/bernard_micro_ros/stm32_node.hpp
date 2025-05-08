// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _STM32_NODE_HPP
#define _STM32_NODE_HPP

#include <bernard_micro_ros/macros.hpp>
#include <bernard_sensors.hpp>
#include <geometry_msgs/msg/accel.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>

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
  rcl_publisher_t pubFootL;
  rcl_publisher_t pubFootR;
  rcl_publisher_t pubQuat;
  rcl_publisher_t pubGyro;
  rcl_publisher_t pubAcc;
  rcl_publisher_t pubStatus;

  // Status fields
  rcl_ret_t retFootL;
  rcl_ret_t retFootR;
  rcl_ret_t retQuat;
  rcl_ret_t retGyro;
  rcl_ret_t retAcc;
  rcl_ret_t retStatus;
  rcl_ret_t retTimer;
  rcl_ret_t retNode;
  rcl_ret_t retExecutor;

  // Message objects
  std_msgs__msg__UInt32 msgFootL;
  std_msgs__msg__UInt32 msgFootR;
  geometry_msgs__msg__Quaternion msgQuat;
  geometry_msgs__msg__Vector3 msgGyro;
  geometry_msgs__msg__Accel msgAcc;
  std_msgs__msg__UInt8 msgStatus;

  static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif // _STM32_NODE_HPP
