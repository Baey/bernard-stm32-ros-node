// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _GUI_HPP
#define _GUI_HPP

#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <bernard_types.hpp>

#include <bernard_wiring.hpp>

/// @brief Available screen types.
typedef enum {
  STATUS,
  IMU_DATA,
} ScreenType;

class BernardGUI {
public:
  BernardGUI(Adafruit_ST7735 *tft, HardwareTimer *screenRefreshTimer,
             BernardStatus *robotStatus);

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
  BernardStatus *currentRobotStatus;
  BernardStatus *previousRobotStatus;
};

#endif // _GUI_HPP
