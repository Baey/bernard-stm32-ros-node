// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <bernard_sensors.hpp>

BernardSensors::BernardSensors(Adafruit_BNO055 *imu, BernardStatus_t *status, BernardGUI *gui, uint32_t lFootContactPin,
                               uint32_t rFootContactPin)
    : imu(imu), status(status), gui(gui), lFootContactPin(lFootContactPin),
      rFootContactPin(rFootContactPin), quat(), linearAcc(), angularAcc(),
      gyro(), footContactLValue(), footContactRValue() {
  sensorTimer = new HardwareTimer(TIM3);
  pingImuTimer = new HardwareTimer(TIM6);
}

IMUStatus_t BernardSensors::initSensors() {
  gui->logMessage("Initializing IMU...");
  status->IMUStatus = initIMU();
  sensorTimer->setOverflow(100, HERTZ_FORMAT);
  sensorTimer->attachInterrupt([this]() { this->readSensorTimerCallback(); });
  sensorTimer->resume();

  pinMode(lFootContactPin, INPUT);
  pinMode(rFootContactPin, INPUT);

  pingImuTimer->setOverflow(1, HERTZ_FORMAT);
  pingImuTimer->attachInterrupt([this]() { this->imuStatusTimerCallback(); });
  pingImuTimer->resume();

  return status->IMUStatus;
}

IMUStatus_t BernardSensors::initIMU() {
  if (!imu->begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    gui->logMessage("IMU not found!");
    return IMU_OFFLINE;
  } else {
    gui->logMessage("IMU found!");
    delay(1000);
    gui->logMessage("Setting XTAL..");
    delay(100);
    /* Use external crystal for better accuracy */
    imu->setExtCrystalUse(true);

    return IMU_ONLINE;
  }
}

imu::Quaternion BernardSensors::readQuaternion() { return imu->getQuat(); }

imu::Vector<3> BernardSensors::readLinearAcceleration() {
  return imu->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

imu::Vector<3> BernardSensors::readAccelerometer() {
  return imu->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}
imu::Vector<3> BernardSensors::readGyroscope() {
  return imu->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

std::vector<uint32_t> BernardSensors::readFootPressure() {
  return {analogRead(lFootContactPin), analogRead(rFootContactPin)};
}

void BernardSensors::readSensorTimerCallback() {
  quat = readQuaternion();
  linearAcc = readLinearAcceleration();
  angularAcc = readAccelerometer();
  gyro = readGyroscope();

  std::vector<uint32_t> footContact = readFootPressure();
  footContactLValue = 0.9 * footContactLValue + 0.1 * footContact[0];
  footContactRValue = 0.9 * footContactRValue + 0.1 * footContact[1];
}

void BernardSensors::imuStatusTimerCallback() {
  imu->getSystemStatus(&imuSystemStatus, &imuSelfTestResult, &imuSystemError);
  status->IMUSystemStatus = static_cast<IMUSystemStatus_t>(imuSystemStatus);
  status->IMUSelfTestResult = static_cast<IMUSelfTestStatus_t>(imuSelfTestResult);
  status->IMUSystemError = static_cast<IMUErrorStatus_t>(imuSystemError);
  
  status->euler = imu->getVector(Adafruit_BNO055::VECTOR_EULER);
}

