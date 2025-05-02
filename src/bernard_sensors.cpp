#include <bernard_sensors.hpp>

BernardSensors::BernardSensors(Adafruit_BNO055 *imu, uint32_t lFootContactPin, uint32_t rFootContactPin)
    : imu(imu), lFootContactPin(lFootContactPin), rFootContactPin(rFootContactPin), quat(),
    linearAcc(), angularAcc(), gyro(), footContactLValue(), footContactRValue() 
{
    sensorTimer = new HardwareTimer(TIM3);
}

bool BernardSensors::initSensors()
{
    bool imuOnline;

    if (!imu->begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      imuOnline = false;
    } else {
      delay(1000);
      /* Use external crystal for better accuracy */
      imu->setExtCrystalUse(true);

      imuOnline = true;
    }

    sensorTimer->setOverflow(100, HERTZ_FORMAT);
    sensorTimer->attachInterrupt([this]() { this->timerCallback(); });
    sensorTimer->resume();

    return imuOnline;
}

imu::Quaternion BernardSensors::readQuaternion()
{
    return imu->getQuat();
}

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

std::vector<uint32_t> BernardSensors::readFootPressure()
{
    return {analogRead(lFootContactPin), analogRead(rFootContactPin)};
}

void BernardSensors::timerCallback()
{
    quat = readQuaternion();
    linearAcc = readLinearAcceleration();
    angularAcc = readAccelerometer();
    gyro = readGyroscope();
    
    std::vector<uint32_t> footContact = readFootPressure();
    footContactLValue = footContact[0];
    footContactRValue = footContact[1];
}