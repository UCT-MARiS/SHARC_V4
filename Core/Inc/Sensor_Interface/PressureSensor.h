/**
 * @file PressureSensor.h
 * @author MichaelNoyce (NYCMIC002@myuct.ac.za)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

// PressureSensor.h
#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#include "ISensor.h"

class PressureSensor : public ISensor {
public:
    ~PressureSensor() override {}
    size_t getBufferSize() const override {
        return 256; // Example buffer size for Pressure sensor
    }
    const char* getCommunicationInterface() const override {
        return "SPI";
    }
    int getSamplingRate() const override {
        return 5; // Example sampling rate in Hz
    }
};

#endif // PressureSENSOR_H
