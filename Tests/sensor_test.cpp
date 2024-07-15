/**
 * @file sensor_test.cpp
 * @author MichaelNoyce (NYCMIC002@myuct.ac.za)
 * @brief Test the sensor classes and factory
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "mock_sensor.h"
#include "SensorFactory.h"

// Test the TemperatureSensor class
TEST(TemperatureSensorTest, BufferSize) {
    TemperatureSensor sensor;
    EXPECT_EQ(sensor.getBufferSize(), 128);
}

TEST(TemperatureSensorTest, CommunicationInterface) {
    TemperatureSensor sensor;
    EXPECT_STREQ(sensor.getCommunicationInterface(), "I2C");
}

TEST(TemperatureSensorTest, SamplingRate) {
    TemperatureSensor sensor;
    EXPECT_EQ(sensor.getSamplingRate(), 10);
}

// Test the PressureSensor class
TEST(PressureSensorTest, BufferSize) {
    PressureSensor sensor;
    EXPECT_EQ(sensor.getBufferSize(), 256);
}

TEST(PressureSensorTest, CommunicationInterface) {
    PressureSensor sensor;
    EXPECT_STREQ(sensor.getCommunicationInterface(), "SPI");
}

TEST(PressureSensorTest, SamplingRate) {
    PressureSensor sensor;
    EXPECT_EQ(sensor.getSamplingRate(), 5);
}

// Test the SensorFactory class
TEST(SensorFactoryTest, CreateTemperatureSensor) {
    ISensor* sensor = SensorFactory::createSensor(SensorFactory::TEMPERATURE);
    ASSERT_NE(sensor, nullptr);
    EXPECT_EQ(sensor->getBufferSize(), 128);
    EXPECT_STREQ(sensor->getCommunicationInterface(), "I2C");
    EXPECT_EQ(sensor->getSamplingRate(), 10);
}

TEST(SensorFactoryTest, CreatePressureSensor) {
    ISensor* sensor = SensorFactory::createSensor(SensorFactory::PRESSURE);
    ASSERT_NE(sensor, nullptr);
    EXPECT_EQ(sensor->getBufferSize(), 256);
    EXPECT_STREQ(sensor->getCommunicationInterface(), "SPI");
    EXPECT_EQ(sensor->getSamplingRate(), 5);
}