// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <Arduino.h>
#include <bernard_micro_ros/stm32_node.hpp>

// Definition of the static member.
STM32Node *STM32Node::instance = nullptr;

STM32Node::STM32Node(BernardSensors &sensors)
    : support(), node(), timer(), executor(), allocator(), pubFootL(),
      pubFootR(), pubQuat(), pubGyro(), pubAcc(), pubStatus(), msgFootL(),
      msgFootR(), msgQuat(), msgGyro(), msgAcc(), msgStatus(),
      sensors(&sensors) {}

/// @brief Destructor that destroys all created ROS entities.
STM32Node::~STM32Node() { destroyEntities(); }

/// @brief Creates all necessary ROS entities.
/// @return true on success.
bool STM32Node::createEntities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

  RCCHECK(rclc_node_init_default(&this->node, "stm32_publisher_rclc", "",
                                 &this->support));

  // Foot contact publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &pubFootL, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "foot_contact_l"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pubFootR, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "foot_contact_r"));

  // IMU publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &pubQuat, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion), "quat"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pubGyro, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "gyro"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pubAcc, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Accel),
      "acc"));

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
  STM32Node::instance->retFootL = rcl_publisher_fini(&pubFootL, &node);
  STM32Node::instance->retFootR = rcl_publisher_fini(&pubFootR, &node);
  STM32Node::instance->retQuat = rcl_publisher_fini(&pubQuat, &node);
  STM32Node::instance->retGyro = rcl_publisher_fini(&pubGyro, &node);
  STM32Node::instance->retAcc = rcl_publisher_fini(&pubAcc, &node);
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

    std::vector<uint32_t> footPressure = node->sensors->getFootPressure();
    imu::Quaternion quat = node->sensors->getQuaternion();
    imu::Vector<3> linearAcc = node->sensors->getLinearAcceleration();
    imu::Vector<3> angularAcc = node->sensors->getAccelerometer();
    imu::Vector<3> gyro = node->sensors->getGyroscope();

    node->msgFootL.data = footPressure[0];
    node->msgFootR.data = footPressure[1];

    node->msgQuat.w = quat.w();
    node->msgQuat.x = quat.x();
    node->msgQuat.y = quat.y();
    node->msgQuat.z = quat.z();

    node->msgAcc.linear.x = linearAcc.x();
    node->msgAcc.linear.y = linearAcc.y();
    node->msgAcc.linear.z = linearAcc.z();
    node->msgAcc.angular.x = angularAcc.x();
    node->msgAcc.angular.y = angularAcc.y();
    node->msgAcc.angular.z = angularAcc.z();

    node->msgGyro.x = gyro.x();
    node->msgGyro.y = gyro.y();
    node->msgGyro.z = gyro.z();

    node->msgStatus.data = 1;

    node->retFootL = rcl_publish(&STM32Node::instance->pubFootL,
                                 &STM32Node::instance->msgFootL, NULL);
    node->retFootR = rcl_publish(&STM32Node::instance->pubFootR,
                                 &STM32Node::instance->msgFootR, NULL);
    node->retQuat = rcl_publish(&STM32Node::instance->pubQuat,
                                &STM32Node::instance->msgQuat, NULL);
    node->retGyro = rcl_publish(&STM32Node::instance->pubGyro,
                                &STM32Node::instance->msgGyro, NULL);
    node->retAcc = rcl_publish(&STM32Node::instance->pubAcc,
                               &STM32Node::instance->msgAcc, NULL);
    node->retStatus = rcl_publish(&STM32Node::instance->pubStatus,
                                  &STM32Node::instance->msgStatus, NULL);
  }
}
