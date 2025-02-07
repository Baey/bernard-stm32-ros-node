#ifndef _IMU_UTILS_HPP
#define _IMU_UTILS_HPP

#include <Adafruit_BNO055.h>

/* Board layout:
       +----------+
       |         *| RST   PITCH  ROLL  HEADING
   ADR |*        *| SCL
   INT |*        *| SDA     ^            /->
   PS1 |*        *| GND     |            |
   PS0 |*        *| 3VO     Y    Z-->    \-X
       |         *| VIN
       +----------+
*/

void imuDetailsToSerial(Adafruit_BNO055 imu);

void quatToSerial(imu::Quaternion quat);

#endif // _IMU_UTILS_HPP
