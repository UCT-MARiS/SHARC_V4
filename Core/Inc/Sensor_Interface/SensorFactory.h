/**
 * @file SensorFactory.cpp
 * @author MichaelNoyce (NYCMIC002@myuct.ac.za)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef SENSORFACTORY_H
#define SENSORFACTORY_H

#include "ISensor.h"
#include "TemperatureSensor.h"
#include "PressureSensor.h"

class SensorFactory {
public:
    enum SensorType {
        TEMPERATURE,
        PRESSURE
    };

    static ISensor* createSensor(SensorType type) {
        switch (type) {
            case TEMPERATURE:
                return &tempSensor;
            case PRESSURE:
                return &pressureSensor;
            default:
                return nullptr;
        }
    }

private:
    static TemperatureSensor tempSensor;
    static PressureSensor pressureSensor;
};

// Definitions of static members
TemperatureSensor SensorFactory::tempSensor;
PressureSensor SensorFactory::pressureSensor;

#endif // SENSORFACTORY_H
