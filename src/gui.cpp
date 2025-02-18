#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <gui.hpp>

/// Construct a new BipedalRobotGUI object.
BipedalRobotGUI::BipedalRobotGUI(Adafruit_ST7735 *tft, HardwareTimer *screenRefreshTimer, RobotStatus *robotStatus)
    : tft(tft), screenRefreshTimer(screenRefreshTimer), currentRobotStatus(robotStatus), previousRobotStatus() {}

/// @brief Initialize the GUI and start the screen refresh timer.
void BipedalRobotGUI::setupGUI()
{
    currentScreen = STATUS;
    nextScreen = STATUS;
    previousRobotStatus = currentRobotStatus;
    SPI.begin();
    tft->initR(INITR_BLACKTAB);
    tft->fillScreen(ST77XX_BLACK);
    tft->setRotation(1);
    tft->setAddrWindow(0, 0, this->tft->width(), this->tft->height());
    tft->setTextColor(ST77XX_WHITE);
    tft->setTextSize(1);

    screenRefreshTimer->setOverflow(1, HERTZ_FORMAT);
    screenRefreshTimer->attachInterrupt(std::bind(&BipedalRobotGUI::drawScreen, this));
    screenRefreshTimer->resume();
}

/// @brief Reset the screen to its default state.
void BipedalRobotGUI::resetScreen()
{
    tft->fillScreen(ST77XX_BLACK);
    tft->setTextColor(ST77XX_WHITE);
    tft->setTextSize(1);
}

/// @brief Refresh the screen by checking if a screen change is requested.
void BipedalRobotGUI::drawScreen()
{
    if (currentScreen != nextScreen)
    {
        resetScreen();
        currentScreen = nextScreen;
        return;
    }
    switch (currentScreen)
    {
    case STATUS:
        drawStatusScreen();
        break;
    case IMU_DATA:
        // drawImuDataScreen();
        break;
    }
}

/// @brief Draw the status screen, displaying IMU and ROS statuses.
void BipedalRobotGUI::drawStatusScreen()
{
    previousRobotStatus = currentRobotStatus;
    tft->fillRect(30, 10, 100, 50, ST77XX_BLACK);
    tft->setCursor(10, 10);
    tft->print("IMU: ");
    if (this->currentRobotStatus->IMUOnline)
    {
        tft->setTextColor(ST77XX_GREEN);
        tft->print("ONLINE");
    }
    else
    {
        tft->setTextColor(ST77XX_RED);
        tft->print("OFFLINE");
    }
    tft->setCursor(10, 20);
    tft->setTextColor(ST77XX_WHITE);
    tft->print("ROS: ");
    switch (this->currentRobotStatus->ROSStatus)
    {
    case WAITING_AGENT:
        tft->setTextColor(ST77XX_YELLOW);
        tft->print("WAITING_AGENT");
        break;
    case AGENT_AVAILABLE:
        tft->setTextColor(ST77XX_GREEN);
        tft->print("AGENT_AVAILABLE");
        break;
    case AGENT_CONNECTED:
        tft->setTextColor(ST77XX_GREEN);
        tft->print("AGENT_CONNECTED");
        break;
    case AGENT_DISCONNECTED:
        tft->setTextColor(ST77XX_RED);
        tft->print("AGENT_DISCONNECTED");
        break;
    }
    tft->setTextColor(ST77XX_WHITE);
}