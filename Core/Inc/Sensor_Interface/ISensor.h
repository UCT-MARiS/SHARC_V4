// ISensor.h
#ifndef ISENSOR_H
#define ISENSOR_H

class ISensor {
public:
    virtual ~ISensor() {}
    virtual size_t getBufferSize() const = 0;
    virtual const char* getCommunicationInterface() const = 0;
    virtual int getSamplingRate() const = 0;
};

#endif // ISENSOR_H