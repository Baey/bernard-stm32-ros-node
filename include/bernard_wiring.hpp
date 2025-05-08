// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _BERNARD_WIRING_HPP
#define _BERNARD_WIRING_HPP

/// @brief Pin definitions for the TFT display.
#define TFT_CS PB6
#define TFT_RST PB7
#define TFT_DC PC6
#define TFT_MOSI PA7
#define TFT_MISO PA6
#define TFT_SCLK PA5
#define TFT_DIN TFT_MOSI

/// @brief Pin definitions for the feet pressure sensors.
#define L_FOOT_ANALOG_PRESSURE_SENSOR PC0

/// @brief Pin definitions for the IMU.
#define IMU_SDA PB9
#define IMU_SCL PB8

#endif // _BERNARD_WIRING_HPP
