// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __BCBBATTERY_H__
#define __BCBBATTERY_H__

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IBattery.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/SerialInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class BcbBattery : public RateThread, public yarp::dev::IBattery, public DeviceDriver
{
protected:
    yarp::os::Semaphore mutex;

    unsigned short     batteryId;
    short              status;
    double             timeStamp;
    yarp::sig::Vector  data;
    double             battery_charge;
    double             battery_voltage;
    double             battery_current;
    double             battery_temperature;
    std::string        battery_info;
    unsigned char      backpack_status;

    bool logEnable;
    bool verboseEnable;
    bool screenEnable;
    bool debugEnable;
    bool shutdownEnable;

    FILE                *logFile;
    ResourceFinder      rf;
    PolyDriver          driver;
    ISerialDevice       *pSerial;
    char                serial_buff[255];
    char                log_buffer[255];
    std::string         remoteName;
    std::string         localName;

public:
    BcbBattery(int period = 20) : RateThread(period), mutex(1)
    {}


    ~BcbBattery()
    {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    virtual bool getBatteryVoltage     (double &voltage);
    virtual bool getBatteryCurrent     (double &current);
    virtual bool getBatteryCharge      (double &charge);
    virtual bool getBatteryStatus      (int &status);
    virtual bool getBatteryInfo        (std::string &info);
    virtual bool getBatteryTemperature (double &temperature);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    void check_battery_status();
    void notify_message(std::string msg);
    void stop_robot(std::string quit_port);
};


#endif
