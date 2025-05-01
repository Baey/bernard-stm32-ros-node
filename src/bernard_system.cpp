#include <bernard_system.hpp>
#include <bernard_micro_ros/stm32_node.hpp>


BernardSystem::BernardSystem()
    : sensors(), gui(), status(), node()
{
    // Initialize the IMU
    TwoWire *imu_i2c = new TwoWire(IMU_SDA, IMU_SCL);
    imu_i2c->begin();
    Adafruit_BNO055 *bno = new Adafruit_BNO055(55, 0x28, imu_i2c);

    // Create the BernardSensors object
    sensors = new BernardSensors(bno, L_FOOT_ANALOG_PRESSURE_SENSOR, L_FOOT_ANALOG_PRESSURE_SENSOR);

    // Initialize TFT display
    Adafruit_ST7735 *tft = new Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
    HardwareTimer *screenRefreshTimer = new HardwareTimer(TIM3);

    status = new BernardStatus();

    // Create the BernardGUI object
    gui = new BernardGUI(tft, screenRefreshTimer, status);

    // Create the STM32Node object
    node = new STM32Node(*sensors);
}

void BernardSystem::init()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.print("Starting BERNARD system...");
    
    Serial.println("Initializing GUI...");
    // Initialize the GUI
    gui->setupGUI();

    Serial.println("Initializing IMU...");
    // Initialize the IMU
    // status->IMUOnline = sensors->initIMU();

    if (status->IMUOnline)
    {
        Serial.println("IMU is online.");
    }
    else
    {
        Serial.println("IMU is offline.");
    }

    Serial.println("Initializing micro-ROS...");
    // Initialize micro-ROS
    // set_microros_serial_transports(Serial); 
    status->ROSStatus = WAITING_AGENT;

    Serial.println("BERNARD system initialized.");
}