#ifndef _GUI_HPP
#define _GUI_HPP

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BNO055.h>
#include <bipedal_types.hpp>

/// @brief Available screen types.
typedef enum
{
    STATUS,
    IMU_DATA,
} ScreenType;

class BipedalRobotGUI
{
public:
    BipedalRobotGUI(Adafruit_ST7735 *tft, HardwareTimer *screenRefreshTimer, RobotStatus *robotStatus);
    
    void setupGUI();
    
    void resetScreen();

    void drawScreen();

    ScreenType currentScreen;
    ScreenType nextScreen;

private:
    void drawStatusScreen();
    // void drawImuDataScreen();
    
    Adafruit_ST7735 *tft;
    HardwareTimer *screenRefreshTimer;
    RobotStatus *currentRobotStatus;
    RobotStatus *previousRobotStatus;
};

#endif // _GUI_HPP
