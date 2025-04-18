#pragma once
#include "vex.h"

using namespace vex;

namespace cholib {
    class trackingWheel {
        private:
        motor_group* motorGroup = nullptr;
        rotation rotationSensor = NULL;
        rotation* detectRotationSensor = nullptr;

        float wheelDiameter;
        float offset;
        float gearRatio;

        public:

        trackingWheel(vex::motor_group* motorGroup, float wheelDiameter, float offset, float gearRatio);
        trackingWheel(int32_t rotationSensor, bool reversed, float wheelDiameter, float offset, float gearRatio);

        void resetEncoders();
        float getOffset();
        float getDistanceTraveled();
    };
}