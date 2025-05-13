// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <micro_ros_platformio.h>

#include <bernard_micro_ros/stm32_node.hpp>
#include <bernard_system.hpp>
#include <bernard_types.hpp>
#include <bernard_wiring.hpp>
#include <gui.hpp>

TwoWire *imu_i2c = new TwoWire(IMU_SDA, IMU_SCL);
// HardwareTimer *imuTimer = new HardwareTimer(TIM2);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, imu_i2c);
imu::Quaternion quat;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
HardwareTimer *screenRefreshTimer = new HardwareTimer(TIM2);
BernardStatus_t bernardStatus;

BernardGUI gui(&tft, screenRefreshTimer, &bernardStatus);
BernardSensors sensors(&bno, &bernardStatus, &gui, L_FOOT_ANALOG_PRESSURE_SENSOR,
                       R_FOOT_ANALOG_PRESSURE_SENSOR);
STM32Node node(sensors);

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(921600);

  gui.initGUI();
  delay(1000);

  gui.logMessage("ROS initialization...");
  delay(1000);
  set_microros_serial_transports(Serial);
  bernardStatus.ROSStatus = WAITING_AGENT;

  bernardStatus.IMUStatus = sensors.initSensors();
  gui.logMessage("Sensors initialized!");
  delay(1000);
  gui.logMessage("Waiting for Kria...");
  delay(1000);
}

void loop(void) {
  switch (bernardStatus.ROSStatus) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(1000, bernardStatus.ROSStatus =
                                 (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                     ? AGENT_AVAILABLE
                                     : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    bernardStatus.ROSStatus =
        (true == node.createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (bernardStatus.ROSStatus == WAITING_AGENT) {
      node.destroyEntities();
    } else {
      gui.setNextScreen(GUI_STATUS);
    }
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(1000, bernardStatus.ROSStatus =
                                 (RMW_RET_OK == rmw_uros_ping_agent(50, 1))
                                     ? AGENT_CONNECTED
                                     : AGENT_DISCONNECTED;);
    if (bernardStatus.ROSStatus == AGENT_CONNECTED) {
      node.spin();
    }
    break;
  case AGENT_DISCONNECTED:
    node.destroyEntities();
    bernardStatus.ROSStatus = WAITING_AGENT;
    break;
  default:
    break;
  }

  if (bernardStatus.ROSStatus == AGENT_CONNECTED) {
    digitalWrite(LED_BUILTIN, 1);
  } else {
    digitalWrite(LED_BUILTIN, 0);
  }
}
