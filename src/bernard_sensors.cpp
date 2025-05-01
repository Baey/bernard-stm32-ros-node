#include <bernard_sensors.hpp>

bool BernardSensors::initIMU()
{
    if (!imu->begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      return false;
    } else {
      delay(1000);
      /* Use external crystal for better accuracy */
      imu->setExtCrystalUse(true);

      return true;
    }
}

imu::Quaternion BernardSensors::getQuaternion()
{
    return imu->getQuat();
}

imu::Vector<3> BernardSensors::getLinearAcceleration()
{
    return imu->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

imu::Vector<3> BernardSensors::getAccelerometer()
{
    return imu->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}
imu::Vector<3> BernardSensors::getGyroscope()
{
    return imu->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

std::vector<uint32_t> BernardSensors::getFootPressure()
{
    std::vector<uint32_t> footContact = {analogRead(footContactL), analogRead(footContactR)};
    return footContact;
}