// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _BERNARD_SYSTEM_HPP
#define _BERNARD_SYSTEM_HPP

#include <bernard_sensors.hpp>
#include <bernard_types.hpp>
#include <bernard_wiring.hpp>
#include <gui.hpp>

// Forward declaration of STM32Node
class STM32Node;

/// @brief Class for handling the BERNARD sensors, gui, status and micro-ROS.
class BernardSystem {
public:
  BernardSystem();

  /// @brief Initializes the BERNARD system.
  void init();

  /// @brief Returns the BERNARD status.
  BernardStatus getStatus() { return *status; }

  /// @brief Returns the BERNARD sensors.
  BernardSensors *getSensors() { return sensors; }

  /// @brief Returns the BERNARD GUI.
  BernardGUI *getGUI() { return gui; }

  /// @brief Returns the BERNARD micro-ROS node.
  STM32Node *getNode() { return node; }

  /// @brief Sets the BERNARD ROS status.
  /// @param status The new ROS status.
  void setROSStatus(microROSStatus_t status) {
    this->status->ROSStatus = status;
  }

  /// @brief Sets the BERNARD IMU status.
  /// @param status The new IMU status.
  void setIMUStatus(bool status) { this->status->IMUOnline = status; }

private:
  BernardSensors *sensors; ///< Pointer to the BERNARD sensors.
  BernardGUI *gui;         ///< Pointer to the BERNARD GUI.
  BernardStatus *status;   ///< Pointer to the BERNARD status.
  STM32Node *node;         ///< Pointer to the STM32 micro-ROS node.
};

#endif // _BERNARD_SYSTEM_HPP
