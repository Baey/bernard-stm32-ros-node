// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <Arduino.h>
#include <bernard_micro_ros/stm32_node.hpp>

// Definition of the static member.
STM32Node *STM32Node::instance = nullptr;

STM32Node::STM32Node(BernardSensors &sensors)
    : support(), node(), timer(), executor(), allocator(), 
      pubFeetPressure(), pubImu(), pubStatus(), msgFeetPressure(),
      msgImu(), msgStatus(), sensors(&sensors) {
        msgFeetPressure.data.size = 2;
      }

/// @brief Destructor that destroys all created ROS entities.
STM32Node::~STM32Node() { destroyEntities(); }

/// @brief Creates all necessary ROS entities.
/// @return true on success.
bool STM32Node::createEntities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

  RCCHECK(rclc_node_init_default(&this->node, "stm32_publisher_rclc", "",
                                 &this->support));

  // Foot contact publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &pubFeetPressure, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
      "feet_pressure"));

  // IMU publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &pubImu, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));

  // Status publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &pubStatus, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "status"));

  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(&this->timer, &this->support,
                                  RCL_MS_TO_NS(timer_timeout),
                                  this->timerCallback));

  STM32Node::instance = this;
  executor = rclc_executor_get_zero_initialized_executor();

  RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 1,
                             &this->allocator));

  RCCHECK(rclc_executor_add_timer(&this->executor, &this->timer));

  return true;
}

/// @brief Destroys all created ROS entities.
void STM32Node::destroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  STM32Node::instance->retFeetPressure = rcl_publisher_fini(&pubFeetPressure, &node);
  STM32Node::instance->retImu = rcl_publisher_fini(&pubImu, &node);
  STM32Node::instance->retStatus = rcl_publisher_fini(&pubStatus, &node);
  STM32Node::instance->retTimer = rcl_timer_fini(&timer);
  STM32Node::instance->retExecutor = rclc_executor_fini(&executor);
  STM32Node::instance->retNode = rcl_node_fini(&node);
  rclc_support_fini(&support);
}

/// @brief Spins the executor to process available callbacks.
void STM32Node::spin() { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); }

/// @brief Timer callback that toggles the boolean messages and publishes them.
/// @param timer The timer object.
/// @param last_call_time The last call time.
void STM32Node::timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (STM32Node::instance != nullptr) {
    STM32Node *node = STM32Node::instance;

    std::vector<uint16_t> footPressure = node->sensors->getFootPressure();
    imu::Quaternion quat = node->sensors->getQuaternion();
    imu::Vector<3> linearAcc = node->sensors->getLinearAcceleration();
    imu::Vector<3> angularAcc = node->sensors->getAccelerometer();
    imu::Vector<3> gyro = node->sensors->getGyroscope();

    node->msgFeetPressure.data.data = &footPressure[0];
    
    node->msgImu.header.stamp.sec = millis() / 1000;
    node->msgImu.orientation.x = quat.x();
    node->msgImu.orientation.y = quat.y();
    node->msgImu.orientation.z = quat.z();
    node->msgImu.orientation.w = quat.w();
    node->msgImu.linear_acceleration.x = linearAcc.x();
    node->msgImu.linear_acceleration.y = linearAcc.y();
    node->msgImu.linear_acceleration.z = linearAcc.z();
    node->msgImu.angular_velocity.x = gyro.x();
    node->msgImu.angular_velocity.y = gyro.y();
    node->msgImu.angular_velocity.z = gyro.z();

    node->msgStatus.data = 1;

    node->retFeetPressure = rcl_publish(&STM32Node::instance->pubFeetPressure,
                                 &STM32Node::instance->msgFeetPressure, NULL);
    node->retImu = rcl_publish(&STM32Node::instance->pubImu,
                                &STM32Node::instance->msgImu, NULL);
    node->retStatus = rcl_publish(&STM32Node::instance->pubStatus,
                                  &STM32Node::instance->msgStatus, NULL);
  }
}
