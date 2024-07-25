/**
 * @file TemperatureSensor.h
 * @author MichaelNoyce (NYCMIC002@myuct.ac.za)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */


// TemperatureSensor.h
#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "ISensor.h"

class TemperatureSensor : public ISensor {
public:
    virtual ~TemperatureSensor() {}
    size_t getBufferSize() const override {
        return 128; // Example buffer size for temperature sensor
    }
    const char* getCommunicationInterface() const override {
        return "I2C";
    }
    int getSamplingRate() const override {
        return 10; // Example sampling rate in Hz
    }
};

#endif // TEMPERATURESENSOR_H
