#pragma once
#include "vex.h"
#include "../api.hpp"

using namespace vex;

namespace cholib {

    namespace Omniwheel {
        constexpr float NEW_275 = 2.75;
        constexpr float OLD_275 = 2.75;
        constexpr float NEW_275_HALF = 2.744;
        constexpr float OLD_275_HALF = 2.74;
        constexpr float NEW_325 = 3.25;
        constexpr float OLD_325 = 3.25;
        constexpr float NEW_325_HALF = 3.246;
        constexpr float OLD_325_HALF = 3.246;
        constexpr float NEW_4 = 4;
        constexpr float OLD_4 = 4.18;
        constexpr float NEW_4_HALF = 3.995;
        constexpr float OLD_4_HALF = 4.175;
    }

    class chassisConfig {
    public:
        chassisConfig(motor_group* leftMotors, motor_group* rightMotors, int32_t inertialPort, float dtWheelDiameter, float dtGearRatio, float dtTrackWidth)
            : leftMotors(leftMotors),
              rightMotors(rightMotors),
              inertialSensor(vex::inertial(inertialPort)),
              dtWheelDiameter(dtWheelDiameter),
              dtGearRatio(dtGearRatio),
              dtTrackWidth(dtTrackWidth)
        {}

        motor_group* leftMotors;
        motor_group* rightMotors;
        inertial inertialSensor;
        float dtWheelDiameter;
        float dtGearRatio;
        float dtTrackWidth;
    };

    class expoDriveConfig {
    public:
        expoDriveConfig(float deadband, float expoCurve)
            : deadband(deadband),
              expoCurve(expoCurve)
        {}

        float deadband;
        float expoCurve;
    };

    class Chassis {
    private:
        vex::mutex mutex;

    protected:
        float distanceTraveled;
        chassisConfig _chassisConfig;
        expoDriveConfig _expoDriveConfig;
        trackingWheel* vTracker;
        trackingWheel* hTracker;

    public:
        Chassis(chassisConfig chassisConfig, expoDriveConfig expoDriveConfig, trackingWheel* vTracker = nullptr, trackingWheel* hTracker = nullptr);

        void setGainScheduling();

        void driveFor(float distance);
        void turnFor(float angle);
        void turnTo(float heading);
        void swingLeft(float angle);
        void swingRight(float angle);
        void turnToPoint(float x, float y);
        void moveToPoint(float x, float y);
        void moveToCoordinates(float x, float y);
    };
}
