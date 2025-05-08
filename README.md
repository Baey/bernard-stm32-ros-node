# STM32 ROS Bernard Node

## Overview

This repository contains the **PlatformIO project** for the micro-ROS node, which handles communication, sensor data reading. It is a submodule of the main [bernard-bipedal-robot](https://github.com/Baey/bernard-bipedal-robot) repository.

The other submodule, [bernard-rl](https://github.com/Baey/BERNARD-RL), provides an IsaacLab extension with an RL environment for training the robot in simulation.

---

## Features

- **Micro-ROS Integration**: Enables real-time communication between the STM32 microcontroller and ROS 2.
- **Sensor Management**: Reads data from IMU and foot pressure sensors.
- **GUI**: Displays system status (IMU, ROS connection) on a TFT screen.
- **Timer-Based Execution**: Periodic sensor updates and data publishing using hardware timers.

---

## Requirements

- **Hardware**:
  - STM32 microcontroller (tested on Nucleo L476RG)
  - Adafruit BNO055 IMU
  - TFT display (ST7735)
  - Analog pressure sensors
- **Software**:
  - [PlatformIO](https://platformio.org/) for building and flashing the firmware.
  - ROS 2 (Foxy or later) with micro-ROS support.

---

## Installation

1. **Clone the Repository**:
   Clone the main repository and initialize submodules:
   ```bash
   git clone --recurse-submodules https://github.com/Baey/bernard-bipedal-robot.git
   cd bernard-bipedal-robot/stm32-ros-bernard-node
   ```

2. **Install PlatformIO**:
   Follow the [PlatformIO installation guide](https://platformio.org/install).

3. **Install Dependencies**:
   PlatformIO will automatically install required libraries (e.g., Adafruit BNO055, Adafruit GFX, micro-ROS).

4. **Build and Upload**:
   Connect your STM32 board and run:
   ```bash
   platformio run --target upload
   ```

---

## Usage

### Micro-ROS Node

The micro-ROS node publishes following information:
- IMU data
- foot pressure sensor readings
- node status

### GUI

The TFT display shows:
- IMU status (online/offline).
- ROS connection status (e.g., waiting for agent, connected).

### ROS Integration

To run the ROS agent for the first time on Linux, follow these steps:

1. **Source the ROS 2 installation**:
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   ```

2. **Create a workspace and download the micro-ROS tools**:
   ```bash
   mkdir microros_ws
   cd microros_ws
   git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
   ```

3. **Update dependencies using rosdep**:
   ```bash
   sudo apt update && rosdep update
   rosdep install --from-paths src --ignore-src -y
   ```

4. **Install pip**:
   ```bash
   sudo apt-get install python3-pip
   ```

5. **Build micro-ROS tools and source them**:
   ```bash
   colcon build
   source install/local_setup.bash
   ```

6. **Run the ROS agent**:
   After sourcing the setup, you can start the micro-ROS agent with the following command:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev [device_port] -b 921600
   ```
   Replace `[device_port]` with the serial port of your STM32 board (e.g., `/dev/ttyUSB0`).

---

## Project Structure

- **`src/`**: Source code for the micro-ROS node and GUI.
- **`include/`**: Header files for the project.
- **`lib/`**: Custom libraries for the project.
- **`test/`**: Unit tests for the firmware.
- **`platformio.ini`**: PlatformIO configuration file.

---

## Development

### Adding Features

1. Modify or add source files in the `src/` directory.
2. Update the `platformio.ini` file if new libraries are required.
3. Test changes using the PlatformIO test framework.

### Testing

Run unit tests with:
```bash
platformio test
```

---

## Related Repositories

- [bernard-bipedal-robot](https://github.com/Baey/bernard-bipedal-robot): Main repository for the project.
- [bernard-rl](https://github.com/Baey/BERNARD-RL): IsaacLab extension for RL environment.

---

## Acknowledgments

This project is part of a master's thesis at AGH University of Krakow.
---
