#include <imu_utils.h>

void imuDetailsToSerial(Adafruit_BNO055 imu) {
    sensor_t sensor;
    imu.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
}

void quatToSerial(imu::Quaternion quat) {
    Serial.print(F("Quaternion: "));
    Serial.print((float)quat.w(), 4);
    Serial.print(F(", "));
    Serial.print((float)quat.x(), 4);
    Serial.print(F(", "));
    Serial.print((float)quat.y(), 4);
    Serial.print(F(", "));
    Serial.print((float)quat.z(), 4);
    Serial.println(F(""));
}