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
4. Make sure to recursively download submodules using the following command in the working directory:
`git submodule update --init --recursive`
5. Install Doxygen
6. If you are using windows, download MSYS2 to use GCC and GDB
7. If you are using Unix (Linux or MacOS), make sure GCC and GDB are installed and up to date. 

## VS Code Hints.
VS Code can run many command line tasks using the following shortcut: Ctrl+Shift+P (Windows)

The .vscode file contains the following useful files:
1. extensions.json 
2. launch.json
3. tasks.json

extensions.json contains all the extensions used by the project as well as several to improve the development 
experience. 

launch.json contains instructions to debug your program. Importantly, you can debug your STM32 remotely using ST Link.
The "Run and Debug" tab is the GUI displaying the instructions given in json form in launch.json. 
You can also use Ctrl+Shift+P to access the commands in launch.json. 

tasks.json is a simple way to make command line argument repeatable and clearer. Use Ctrl+Shift+B. 
Currently, tasks are used to upload the release binary to the STM32 MCU and build the doxygen documentation. 


## Testing
CTest is used to run testing. Alternatively TestMate ++ can be used. CTest currently works with the "Build Server" configure preset. 
GTest and GMock are used to write tests. Where possible, please use test driven development! ( [TDD](https://en.wikipedia.org/wiki/Test-driven_development) )


## CMake
The project uses CMake as its build system. The project is configured to run both on the STM32L4R5 and your local system. 
This is done by controlling the CMake configure presets in the CMakePresets.json file. For example, "Build Server" is for your 
local system and "Debug", "RelWithDebInfo" and "Release" are targeted at your STM32 MCU. The configuration presets can be selected 
using Ctrl+Shift+P and selecting "CMake: Select Configure Preset"

Once the preset is selected, configure and build the project. 

## STM32 HAL

## Usage

1. Connect the STM32L4 microcontroller to your PC
2. Power on the board and ensure proper connections
3. Chose the correct CMake configuration and build (e.g. Debug)
4. Use the "Upload Release to STM32" (see tasks.json)
5. OR see a debugging option in launch.json

## Contributing

Contributions are welcome! Please follow the guidelines in the [CONTRIBUTING.md](CONTRIBUTING.md) file.

## License

This project is licensed under the [MIT License](LICENSE).
