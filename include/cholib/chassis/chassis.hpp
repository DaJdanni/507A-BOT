#pragma once
#include "vex.h"
#include "api.hpp"

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
            chassisInfo(motor_group* leftMotors, motor_group* rightMotors, int32_t inertialPort, float dtWheelDiameter, float dtGearRatio, float dtTrackWidth) :
            leftMotors(leftMotors),
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
    }

    class expoDriveConfig {
        public:
            expoDriveConfig(float deadband, float expoCurve) :
            deadband(deadband),
            expoCurve(expoCurve)
        {}
        float deadband;
        float expoCurve;
    }

    class mclConfig {
        public:
            mclConfig(int32_t googagagaga, NULL hi, )
    }

    class Chassis {
        private:

        public:


        Chassis(chassisConfig, expoDriveConfig, trackingWheel* vTracker, trackingWheel* hTracker, )
    }
}