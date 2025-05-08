// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _BERNARD_SENSORS_HPP
#define _BERNARD_SENSORS_HPP

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

/// @brief Class with BERNARD sensors.
/// @details This class so far contains the IMU sensor and the foot contact
/// sensors.
class BernardSensors {
public:
  BernardSensors(Adafruit_BNO055 *imu, uint32_t lFootContactPin,
                 uint32_t rFootContactPin);

  ~BernardSensors() {}

  /// @brief Initializes the Bernard sensors.
  /// @return true if the IMU sensor is online.
  bool initSensors();

  /// @brief Reads the IMU quaternion.
  imu::Quaternion readQuaternion();

  /// @brief Reads the IMU linear acceleration in m/s^2.
  /// @details The IMU linear acceleration is the acceleration of the robot in
  /// the x, y and z axes.
  imu::Vector<3> readLinearAcceleration();

  /// @brief Reads the IMU accelerometer data in m/s^2.
  /// @details The IMU accelerometer data is the acceleration of the robot in
  /// the x, y and z axes.
  imu::Vector<3> readAccelerometer();

  /// @brief Reads the IMU gyroscope data in rad/s.
  /// @details The IMU gyroscope data is the angular velocity of the robot in
  /// the x, y and z axes.
  imu::Vector<3> readGyroscope();

  /// @brief Reads analog values from the foot contact sensors.
  /// @details The foot contact sensors are analog sensors that return a value
  /// between 0 and 1023.
  /// @details The foot contact sensors are connected to the analog pins of the
  /// microcontroller.
  /// @details The foot contact sensors are used to detect if the robot is in
  /// contact with the ground.
  /// @return A vector of uint32_t containing the analog values from the left
  /// and right foot contact sensors.
  std::vector<uint32_t> readFootPressure();

  /// @brief Gets the IMU quaternion.
  /// @return The IMU quaternion.
  imu::Quaternion getQuaternion() { return quat; }

  /// @brief Gets the IMU linear acceleration.
  /// @return The IMU linear acceleration.
  imu::Vector<3> getLinearAcceleration() { return linearAcc; }

  /// @brief Gets the IMU accelerometer data.
  /// @return The IMU accelerometer data.
  imu::Vector<3> getAccelerometer() { return angularAcc; }

  /// @brief Gets the IMU gyroscope data.
  /// @return The IMU gyroscope data.
  imu::Vector<3> getGyroscope() { return gyro; }

  /// @brief Gets the foot contact sensors values.
  /// @return A vector of uint32_t containing the analog values from the left
  /// and right foot contact sensors.
  std::vector<uint32_t> getFootPressure() {
    return {footContactLValue, footContactRValue};
  }

  /// @brief Timer callback function.
  /// @details This function is called every 100ms to read the IMU data and the
  /// foot contact sensors.
  void timerCallback();

private:
  Adafruit_BNO055 *imu;
  sensors_event_t imuEvent;
  HardwareTimer *sensorTimer;
  uint32_t lFootContactPin;
  uint32_t rFootContactPin;
  imu::Quaternion quat;
  imu::Vector<3> linearAcc;
  imu::Vector<3> angularAcc;
  imu::Vector<3> gyro;
  uint32_t footContactLValue;
  uint32_t footContactRValue;
};

#endif // _BERNARD_SENSORS_HPP
