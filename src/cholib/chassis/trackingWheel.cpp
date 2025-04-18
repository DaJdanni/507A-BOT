#include "vex.h"

cholib::trackingWheel::trackingWheel(vex::motor_group *motorGroup, float wheelDiameter, float offset, float gearRatio) :
    motorGroup(motorGroup)
{
    std::cout << "TRACKING WHEEL IS USING INTERNAL MOTOR ENCODERS." << std::endl;
    this->wheelDiameter = wheelDiameter;
    this->offset = offset;
    this->gearRatio = gearRatio;   
}

cholib::trackingWheel::trackingWheel(int32_t rotationSensor, bool reversed, float wheelDiameter, float offset, float gearRatio)  : 
    rotationSensor(vex::rotation(rotationSensor, reversed))
{
    if (this->rotationSensor.installed() != true) {
        std::cout << "ROTATION SENSOR NOT INSTALLED. CHECK THE PORT." << std::endl;
    }

    std::cout << "TRACKING WHEEL IS USING ROTATION SENSOR." << std::endl;
    this->wheelDiameter = wheelDiameter;
    this->offset = offset;
    this->gearRatio = gearRatio;
    this->detectRotationSensor = &this->rotationSensor;
}

void cholib::trackingWheel::resetEncoders() {
    std::cout << "Reset encoders" << std::endl;
    if (this->detectRotationSensor != nullptr) {
        this->rotationSensor.resetPosition();
    } else if (this->motorGroup != nullptr) {
        this->motorGroup->resetPosition();
    }
}

float cholib::trackingWheel::getOffset() {
    return this->offset;
}

float cholib::trackingWheel::getDistanceTraveled() {
    if (this->detectRotationSensor != nullptr) {
        //return this->rotationSensor.position(vex::rotationUnits::deg);
       return to_wheel_travel(this->rotationSensor.position(vex::rotationUnits::deg), this->wheelDiameter, this->gearRatio);
    } else if (this->motorGroup != nullptr) {
        //return this->motorGroup->position(vex::rotationUnits::deg);
       return to_wheel_travel(this->motorGroup->position(vex::rotationUnits::deg), this->wheelDiameter, this->gearRatio);
    }

    return -999999;
}