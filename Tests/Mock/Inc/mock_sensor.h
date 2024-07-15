#ifndef MOCKSENSOR_H
#define MOCKSENSOR_H

#include <gmock/gmock.h>
#include "ISensor.h"

class MockSensor : public ISensor {
public:
    MOCK_METHOD(size_t, getBufferSize, (), (const, override));
    MOCK_METHOD(const char*, getCommunicationInterface, (), (const, override));
    MOCK_METHOD(int, getSamplingRate, (), (const, override));
};

#endif // MOCKSENSOR_H
