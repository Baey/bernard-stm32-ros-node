// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <bernard_sensors.hpp>

BernardSensors::BernardSensors(Adafruit_BNO055 *imu, BernardStatus_t *status, BernardGUI *gui, uint32_t lFootContactPin,
                               uint32_t rFootContactPin)
    : imu(imu), status(status), gui(gui), lFootContactPin(lFootContactPin),
      rFootContactPin(rFootContactPin), quat(), linearAcc(), angularAcc(),
      gyro(), footContactLValue(), footContactRValue()
{
  highFreqTimer = new HardwareTimer(TIM3);
  lowFreqTimer = new HardwareTimer(TIM4);
  pingImuTimer = new HardwareTimer(TIM6);

}

IMUStatus_t BernardSensors::initSensors()
{
  gui->logMessage("Initializing IMU...");
  status->IMUStatus = initIMU();
  highFreqTimer->setOverflow(100, HERTZ_FORMAT);
  highFreqTimer->attachInterrupt([this]()
                                  { this->readHighFrequencyTimerCallback(); });
  highFreqTimer->resume();

  pinMode(lFootContactPin, INPUT);
  pinMode(rFootContactPin, INPUT);

  lowFreqTimer->setOverflow(1, HERTZ_FORMAT);
  lowFreqTimer->attachInterrupt([this]()
                                 { this->readLowFrequencyTimerCallback(); });
  lowFreqTimer->resume();

  pingImuTimer->setOverflow(1, HERTZ_FORMAT);
  pingImuTimer->attachInterrupt([this]()
                                { this->imuStatusTimerCallback(); });
  pingImuTimer->resume();

  return status->IMUStatus;
}

IMUStatus_t BernardSensors::initIMU()
{
  if (!imu->begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    gui->logMessage("IMU not found!");
    return IMU_OFFLINE;
  }
  else
  {
    gui->logMessage("IMU found!");
    delay(1000);
    gui->logMessage("Setting XTAL..");
    delay(100);
    /* Use external crystal for better accuracy */
    imu->setExtCrystalUse(true);
    // imu->setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P5);
    // imu->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P5);

    return IMU_ONLINE;
  }
}

imu::Quaternion BernardSensors::readQuaternion() { return imu->getQuat(); }

imu::Vector<3> BernardSensors::readLinearAcceleration()
{
  return imu->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

imu::Vector<3> BernardSensors::readAccelerometer()
{
  return imu->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}
imu::Vector<3> BernardSensors::readGyroscope()
{
  return imu->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

int8_t BernardSensors::readTemperature() { return imu->getTemp(); }

std::array<uint16_t, 2> BernardSensors::readFootPressure()
{
  return {static_cast<uint16_t>(analogRead(lFootContactPin)), static_cast<uint16_t>(analogRead(rFootContactPin))};
}

void BernardSensors::readHighFrequencyTimerCallback()
{
  quat = readQuaternion();
  linearAcc = readLinearAcceleration();
  gyro = readGyroscope();

  std::array<uint16_t, 2> footContact = readFootPressure();
  footContactLValue = 0.9 * footContactLValue + 0.1 * footContact[0];
  footContactRValue = 0.9 * footContactRValue + 0.1 * footContact[1];
}

void BernardSensors::readLowFrequencyTimerCallback()
{
  temp = readTemperature();
}

void BernardSensors::imuStatusTimerCallback()
{
  imu->getSystemStatus(&imuSystemStatus, &imuSelfTestResult, &imuSystemError);
  status->IMUSystemStatus = static_cast<IMUSystemStatus_t>(imuSystemStatus);
  status->IMUSelfTestResult = static_cast<IMUSelfTestStatus_t>(imuSelfTestResult);
  status->IMUSystemError = static_cast<IMUErrorStatus_t>(imuSystemError);

  status->euler = imu->getVector(Adafruit_BNO055::VECTOR_EULER);
}
