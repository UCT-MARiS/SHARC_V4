# SHARC V4

The SHARC V4 firmware is intended to be deployed on SCALE 2025.
It is a waves-in-ice instruments intended to be used to determine ice drift and
wave attenuation rates in the Antarctic Marginal Ice Zone.

## Version History
The version history of the SHARC Buoy is as follows:

### 1.0 2018
Initial prototype proposed by Prof Amit Mishra, Jarryd Son and
Marcello Vichi. Envisioned as a multi-sensor platform.

### 1.1 2019
First prototype developed by Robyn Verrinder, Jamie Jacobson
and Justin Pead. Ice-drift and environmental sensor platform.
### 1.1 2019
First version tested on the SCALE cruise 2019. 2 units deployed. 1 lasted
for ∼1 hour.
### 2.0 (1.2) 2019
Second prototype developed by Robyn Verrinder, Jamie Jacobson
and Justin Pead. Integrated IMU (MPU6050) as proof of concept.
### 2.0 2021
Second version deployed on the SANAE summer relief mission
and from the RV Polarstern. Deployment lasted ∼1 week.
### 3.0 2022
Third prototype developed by Robyn Verrinder, Michael Noyce,
Lawrence Stanton and Justin Pead. Integrated waves-in-ice
measurement and high-frequency local storage.
### 3.0 2022
Third version deployed on SCALE winter cruise 2022. 6 units deployed.
4 function as waves-in-ice devices, 2 as ice-drift devices.

## Description

This project is aimed at developing a firmware for the STM32L4 microcontroller using the FreeRTOS operating system.
As the device is deployed in situ in an extreme environment code is developed for with reliability AND robustness.

## Features

- Utilizes the power-efficient STM32L4 microcontroller
- Implements the FreeRTOS operating system for task scheduling
- Device driver are included as github submodules
- Code coverage with GTest
- Mocking of embedded interfaces with GMock
- Doxygen documentation

## Installation

1. Download VS Code
2. Download the suggested extensions from the extensions.json (if not done automatically)
3. Clone the repository: `git clone https://github.com/MichaelNoyce/SHARC_V4.git`
4. Make sure to recursively download submodules
5. Install Doxygen
6. If you are using windows, download MSYS2 to use GCC and GDB

## Usage

1. Connect the STM32L4 microcontroller to your PC
2. Power on the board and ensure proper connections
3. Chose the correct CMake configuration and build
4. Use the "Upload Release to STM32" (see tasks.json)
5. OR see a debugging option in launch.json

## Contributing

Contributions are welcome! Please follow the guidelines in the [CONTRIBUTING.md](CONTRIBUTING.md) file.

## License

This project is licensed under the [MIT License](LICENSE).
