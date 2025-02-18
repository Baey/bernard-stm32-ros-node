#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <micro_ros_platformio.h>

#include <imu_utils.h>
#include <gui.hpp>
#include <bipedal_types.hpp>
#include <bipedal_wiring.hpp>
#include <bipedal_micro_ros/stm32_node.hpp>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
HardwareTimer *imuTimer = new HardwareTimer(TIM2);
imu::Quaternion quat;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
HardwareTimer *screenRefreshTimer = new HardwareTimer(TIM3);
RobotStatus robotStatus;

BipedalRobotGUI gui(&tft, screenRefreshTimer, &robotStatus);
STM32Node node;

void imuTimerInterrupt()
{
  sensors_event_t event;
  bno.getEvent(&event);
  quat = bno.getQuat();
  // quatToSerial(quat);
}

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  gui.setupGUI();

  set_microros_serial_transports(Serial);
  robotStatus.ROSStatus = WAITING_AGENT;

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    robotStatus.IMUOnline = false;
  }
  else
  {
    robotStatus.IMUOnline = true;
  }

  delay(1000);

  /* Use external crystal for better accuracy */
  // bno.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  // imuDetailsToSerial(bno);

  // imuTimer->setOverflow(100, HERTZ_FORMAT);
  // imuTimer->attachInterrupt(imuTimerInterrupt);
  // imuTimer->resume();
}

void loop(void)
{
  switch (robotStatus.ROSStatus)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(1000, robotStatus.ROSStatus = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    robotStatus.ROSStatus = (true == node.createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (robotStatus.ROSStatus == WAITING_AGENT)
    {
      node.destroyEntities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, robotStatus.ROSStatus = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (robotStatus.ROSStatus == AGENT_CONNECTED)
    {
      node.spin();
    }
    break;
  case AGENT_DISCONNECTED:
    node.destroyEntities();
    robotStatus.ROSStatus = WAITING_AGENT;
    break;
  default:
    break;
  }

  if (robotStatus.ROSStatus == AGENT_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, 1);
  }
  else
  {
    digitalWrite(LED_BUILTIN, 0);
  }
}