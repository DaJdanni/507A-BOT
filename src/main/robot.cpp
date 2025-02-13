#include "vex.h"

template <class F>
vex::task launch_task(F&& function) {
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

/**
 * @brief Initalize a new Robot.
 * 
 * @param drivetrainWheelDiameter The wheel diameter of the drivetrain wheels
 * @param drivetrainGearRatio The gear ratio for the drivetrain
 * @param drivetrainTrackWidth The trackwidth of the chassis
 * @param LeftSide The motor group object that contains the left side motors for the drivetrain
 * @param RightSide The motor group object that contains the right side motors for the drivetrain
 * @param InertialPort The PORT for the Inertial Sensor [SET TO "nullptr" IF NO INERTIAL SENSOR]
 * @param RightTracker The Tracking Wheel Object that contains the data for the right tracker
 * @param BackTracker The Tracking Wheel Object that contains the data for the back tracker [SET TO "nullptr" IF NO BACK TRACKER]
 * @param Linear The Linear PID Settings for the robot
 * @param Angular The Angular PID Settings for the robot
 * @param AntiDrift The Anti Drift PID Settings for the robot
*/
Bucees::Robot::Robot(float drivetrainWheelDiameter, float drivetrainGearRatio, float drivetrainTrackWidth, vex::motor_group* LeftSide, vex::motor_group* RightSide, int32_t InertialPort, Bucees::TrackingWheel* RightTracker, Bucees::TrackingWheel* BackTracker, FAPIDController* Linear, FAPIDController* Angular, FAPIDController* AntiDrift) :
    drivetrainWheelDiameter(drivetrainWheelDiameter),
    drivetrainGearRatio(drivetrainGearRatio),
    drivetrainTrackWidth(drivetrainTrackWidth),
    LeftSide(LeftSide),
    RightSide(RightSide),
    InertialSensor(vex::inertial(InertialPort)),
    RightTracker(RightTracker),
    BackTracker(BackTracker),
    Linear(Linear),
    Angular(Angular),
    AntiDrift(AntiDrift)
{
    if (this->InertialSensor.installed() != true) {
        std::cout << "INERTIAL SENSOR NOT INSTALLED. CHECK THE PORT." << std::endl;
    }
    std::cout << "BACK TRACKER EXIST." << std::endl;
} // BACK TRACKER EXISTS
Bucees::Robot::Robot(float drivetrainWheelDiameter, float drivetrainGearRatio, float drivetrainTrackWidth, vex::motor_group* LeftSide, vex::motor_group* RightSide, int32_t InertialPort, Bucees::TrackingWheel* RightTracker, std::nullptr_t BackTracker, FAPIDController* Linear, FAPIDController* Angular, FAPIDController* AntiDrift) :
    drivetrainWheelDiameter(drivetrainWheelDiameter),
    drivetrainGearRatio(drivetrainGearRatio),
    drivetrainTrackWidth(drivetrainTrackWidth),
    LeftSide(LeftSide),
    RightSide(RightSide),
    InertialSensor(vex::inertial(InertialPort)),
    RightTracker(RightTracker),
    BackTracker(BackTracker),
    Linear(Linear),
    Angular(Angular),
    AntiDrift(AntiDrift)
{
    if (this->InertialSensor.installed() != true) {
        std::cout << "INERTIAL SENSOR NOT INSTALLED. CHECK THE PORT." << std::endl;
    }
    std::cout << "NO BACK TRACKER EXIST." << std::endl;
} // NO BACK TRACKER

/**
 * @brief Get the absolute heading of the inertial sensor using fmodf to constrain it to [0, 360]
*/
float Bucees::Robot::getAbsoluteHeading() {
    return fmodf(InertialSensor.rotation(), 360);
}

/**
 * @brief Control the robot's drivetrain using arcade mode on the controller.
 * 
 * @param LeftJoystickPosition The position of the left joystick
 * @param RightJoystickPosition The position of the right joystick
*/
void Bucees::Robot::arcade(float LeftJoystickPosition, float RightJoystickPosition) {

      if (fabs(LeftJoystickPosition) < 5) { // deadzone
        LeftJoystickPosition = 0;
      } 
      if (fabs(RightJoystickPosition) < 5) { // deadzone
        RightJoystickPosition = 0;
      }

      LeftJoystickPosition *= 0.12;
      RightJoystickPosition *= 0.12;

      LeftSide->spin(vex::directionType::fwd, LeftJoystickPosition + RightJoystickPosition, vex::voltageUnits::volt);
      RightSide->spin(vex::directionType::fwd, LeftJoystickPosition - RightJoystickPosition, vex::voltageUnits::volt);
}

/**
 * @brief Set the odometry offsets and calibrate inertial
*/
void Bucees::Robot::initOdom() {

    float rightOffset = this->RightTracker->getOffset();
    float backOffset = (this->BackTracker != nullptr) ? this->BackTracker->getOffset() : 0;

    this->RightTracker->resetEncoders();
    if (this->BackTracker != nullptr) this->BackTracker->resetEncoders();

    odometry.setOdometry(rightOffset, backOffset);
    reversedOdometry.setOdometry(-rightOffset, -backOffset);

    launch_task([&] {
        while (1) {
            float rightPosition = this->RightTracker->getDistanceTraveled();
            float backPosition = (this->BackTracker != nullptr) ? this->BackTracker->getDistanceTraveled() : 0;

            this->RobotPosition = this->odometry.updatePosition(this->RobotPosition, rightPosition, backPosition, to_rad(getAbsoluteHeading()));
            this->reversedRobotPosition = this->reversedOdometry.updatePosition(this->reversedRobotPosition, -rightPosition, -backPosition, to_rad(getAbsoluteHeading()));
            
            vex::wait(10, vex::msec);
        }
    });
}

/**
 * @brief Reset the odometry values
 */
void Bucees::Robot::resetOdom() {
    this->RightTracker->resetEncoders();
    this->BackTracker->resetEncoders();
   // this->InertialSensor.resetRotation();
    odometry.resetOdometry();
    reversedOdometry.resetOdometry();
    this->setRobotCoordinates({0, 0, 0});
}

/**
 * @brief Set the robots position to a specific coordinate
 * 
 * @param coordinates The coordinates the robot will start at the beginning of the autonomous
*/
void Bucees::Robot::setRobotCoordinates(Bucees::Coordinates coordinates) {
    printf("Set coordinates! \n");
    if (coordinates.theta != 999) this->InertialSensor.setRotation(coordinates.theta, vex::rotationUnits::deg);
    this->odometry.resetOdometry();
    this->reversedOdometry.resetOdometry();
    this->RightTracker->resetEncoders();
    if (this->BackTracker != nullptr) this->BackTracker->resetEncoders();
    coordinates.theta = to_rad(this->InertialSensor.rotation(vex::degrees));
    this->getRobotCoordinates(true, false);
    this->getRobotCoordinates(true, true);
    this->RobotPosition = coordinates;
    this->reversedRobotPosition = coordinates;
}  

/**
 * @brief Returns the robot's coordinates calculated by odometry.
 * 
 * @param radians Whether or not to return the robot coordinates in radians/degrees. [true by default]
 * @param reversed Whether or not to return the robot coordinates as if the drivetrain was reversed. [false by default]
*/
Bucees::Coordinates Bucees::Robot::getRobotCoordinates(bool radians, bool reversed) {

    Bucees::Coordinates modifiedCoordinates = this->RobotPosition;
    Bucees::Coordinates modifiedCoordinatesR = this->reversedRobotPosition;


    if (radians == false) modifiedCoordinates.theta = to_deg(modifiedCoordinates.theta);
    if (radians == false && reversed == true) modifiedCoordinatesR.theta = to_deg(modifiedCoordinatesR.theta);

    if (reversed == false) {
        return modifiedCoordinates;
    } else {
        return modifiedCoordinatesR;
    }
}

/**
 * @brief Waits until the chassis is done with its movement
*/
void Bucees::Robot::waitChassis() {
    wait(10, vex::msec);
    while (distanceTraveled != -1) {
        wait(10, vex::msec);
    }
}

/**
 * @brief Waits until the chassis has traveled a certain distance
 * 
 * @param distance The distance the chassis has traveled [ANY UNITS]
*/
void Bucees::Robot::waitChassis(float distance) {
    wait(10, vex::msec);
    while (distanceTraveled < distance) {
        wait(10, vex::msec);
    }
    wait(20, vex::msec);
}


/**
 * @brief Drive the robot for a certain amount of inches
 * 
 * @param target The amount of inches to drive.
 * @param setings The PID Settings to use for the movement
 * @param antiDrift Whether or not to apply anti drift correction to movement. [default ]
 * @param timeout The amount of time the robot has to complete the action before moving on. [default to 0 meaning no timeout]
 * @param async Determine whether or not to run command in a separate thread. 
*/
void Bucees::Robot::DriveFor(float target, PIDSettings settings, bool antiDrift, float timeout, bool async) {

    if (async == true) {
        launch_task([&] {
            this->DriveFor(target, settings, antiDrift, timeout, false);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    Linear->setGains(settings);
    Linear->setTimeoutTime(timeout);

    Bucees::FAPIDController leftController(
        Linear->settings
    );
    Bucees::FAPIDController rightController(
        Linear->settings
    );

    leftController.setTimeoutTime(timeout);
    rightController.setTimeoutTime(timeout);

 //   LeftSide->resetPosition();
  //  RightSide->resetPosition();

    float previousLPosition = LeftSide->position(vex::rotationUnits::deg);
   // float previousRPosition = this->RightTracker->getDistanceTraveled();

    distanceTraveled = 0;

    float targetRotation = InertialSensor.rotation();

   // printf("previousLPosition: %f \n", previousLPosition);
  //  printf("previousRPosition %f \n", previousRPosition);
    printf("Rotation Before: %f \n", targetRotation);

    while (1) {

        float leftPosition = LeftSide->position(vex::rotationUnits::deg) - previousLPosition;
    //   float rightPosition = this->RightTracker->getDistanceTraveled() - previousRPosition;

        float distanceRotation = targetRotation - InertialSensor.rotation();

        leftPosition = to_wheel_travel(leftPosition, drivetrainWheelDiameter, drivetrainGearRatio);
      //  rightPosition = to_wheel_travel(rightPosition, drivetrainWheelDiameter, drivetrainGearRatio);

        float leftMotorsPower = leftController.calculateMotorPower(target - leftPosition);
     //   float rightMotorsPower = rightController.calculateMotorPower(target - rightPosition);

        float antiDriftPower = antiDrift ? AntiDrift->calculateMotorPower(distanceRotation) : 0;

        distanceTraveled = (leftPosition); // the average the drivetrain has moved

        //printf("error: %f \n", target - (leftPosition));
       // printf("Motor Power: %f \n", );

        if (antiDrift == true) {
            LeftSide->spin(vex::directionType::fwd, leftMotorsPower + antiDriftPower, vex::voltageUnits::volt);
            RightSide->spin(vex::directionType::fwd, leftMotorsPower - antiDriftPower, vex::voltageUnits::volt);
        } else {
            LeftSide->spin(vex::directionType::fwd, leftMotorsPower, vex::voltageUnits::volt);
            RightSide->spin(vex::directionType::fwd, leftMotorsPower, vex::voltageUnits::volt);
        }

        if (leftController.isSettled() == true || rightController.isSettled() == true) break;

        wait(10, vex::msec);
    }

    leftController.reset();
    rightController.reset();
    Linear->reset();
    AntiDrift->reset();

    LeftSide->stop(vex::brakeType::hold);
    RightSide->stop(vex::brakeType::hold);

    distanceTraveled = -1;

    printf("Rotation After: %f \n", InertialSensor.rotation());
    printf("kP: %f, kI: %f, kD: %f\n", settings.kP, settings.kI, settings.kD);

    this->mutex.unlock();

    return;
}

/**
 * @brief Turn the robot for a certain amount of degrees
 * 
 * @param target The amount of degrees to turn for
 * @param settings The PID Settings to use for the movement
 * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
 * @param async Determine whether or not to run command in a separate thread. 
*/
void Bucees::Robot::TurnFor(float target, PIDSettings settings, float timeout, bool async) {

    if (async == true) {
        launch_task([&] {
            this->TurnFor(target, settings, timeout, false);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    Angular->setGains(settings);
    Angular->setTimeoutTime(timeout);

    while (1) {

        float deltaTheta = remainderf(target - InertialSensor.heading(), 360); // scale the error to -180 - 180 turns to take the most efficient routes

        //printf("Rotation: %f \n", InertialSensor.heading());

        distanceTraveled = deltaTheta;

        float motorPower = Angular->calculateMotorPower(deltaTheta);

        LeftSide->spin(vex::directionType::fwd, motorPower, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, -motorPower, vex::voltageUnits::volt);

        if (Angular->isSettled() == true) break;

        wait(10, vex::msec);
    }

    Angular->reset();

    LeftSide->stop(vex::brakeType::hold);
    RightSide->stop(vex::brakeType::hold);

    distanceTraveled = -1;

  //  printf("Turned To: %f \n", InertialSensor.heading());
  //  printf("kP: %f, kI: %f, kD: %f\n", settings.kP, settings.kI, settings.kD);

    this->mutex.unlock();

    return;
}

/**
 * @brief Move to a heading in a hook motion by powering one side of the drivetrain
 * 
 * @param target The amount of degrees to hook to
 * @param leftPower The amount of power to apply to the left side of the drivetrain to allow for longer hooks [0 - 12 range]
 * @param reversed True of false value that determines whether or not to hook going forward/backward
 * @param timeout The amount of time the robot has to complete the action before moving [default to 0 meaning no timeout]
 * @param async Determine whether or not to run command in a separate thread. 
*/
void Bucees::Robot::HookLeft(float target, float leftPower, bool reversed, float timeout, bool async) {

    if (async == true) {
        launch_task([&] {
            this->HookLeft(target, leftPower, reversed, timeout, false);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    Angular->setTimeoutTime(timeout);

    while (1) {

        float deltaTheta = remainderf(target - InertialSensor.rotation(), 360); // scale the error to -180 - 180 turns to take the most efficient routes

        printf("error: %f \n", 0 - deltaTheta);

        distanceTraveled = deltaTheta;

        float motorPower = Angular->calculateMotorPower(deltaTheta);
        float leftPowerConverted = leftPower;

        if (fabs(motorPower) < leftPower) leftPowerConverted = 0;

        motorPower = reversed ? motorPower : -motorPower;
        leftPowerConverted = reversed ? -leftPowerConverted : leftPowerConverted;

        LeftSide->spin(vex::directionType::fwd, leftPowerConverted, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, -motorPower, vex::voltageUnits::volt);

        if (Angular->isSettled() == true) break;

        wait(10, vex::msec);
    }

    Angular->reset();
    LeftSide->stop(vex::brakeType::coast);
    RightSide->stop(vex::brakeType::coast);
    distanceTraveled = -1;

    printf("Hooked Left To: %f \n", InertialSensor.rotation());

    LeftSide->spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
    RightSide->spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);

    wait(20, vex::msec);

    this->mutex.unlock();

    return;
}

/**
 * @brief Move to a heading in a hook motion by powering one side of the drivetrain
 * 
 * @param target The amount of degrees to hook to
 * @param rightPower The amount of power to apply to the right side of the drivetrain to allow for longer hooks [0 - 12 range]
 * @param reversed True of false value that determines whether or not to hook going forward/backward
 * @param timeout The amount of time the robot has to complete the action before moving [default to 0 meaning no timeout]
 * @param async Determine whether or not to run command in a separate thread. 
*/
void Bucees::Robot::HookRight(float target, float rightPower, bool reversed, float timeout, bool async) {

    if (async == true) {
        launch_task([&] {
            this->HookRight(target, rightPower, reversed, timeout, false);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    Angular->setTimeoutTime(timeout);

    while (1) {

        float deltaTheta = remainderf(target - InertialSensor.rotation(), 360); // scale the error to -180 - 180 turns to take the most efficient routes

        distanceTraveled = deltaTheta;

        float motorPower = Angular->calculateMotorPower(deltaTheta);

        float rightPowerConverted = rightPower;

        if (motorPower < rightPower) rightPowerConverted = 0;

        motorPower = reversed ? -motorPower : motorPower;
        rightPowerConverted = reversed ? -rightPowerConverted : rightPowerConverted;

        LeftSide->spin(vex::directionType::fwd, motorPower, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, rightPowerConverted, vex::voltageUnits::volt);

        printf("motorPower: %f \n", motorPower);

        if (Angular->isSettled() == true) break;

        wait(10, vex::msec);
    }

    Angular->reset();
    LeftSide->stop(vex::brakeType::coast);
    RightSide->stop(vex::brakeType::coast);
    distanceTraveled = -1;

    printf("Hooked Right To: %f \n", InertialSensor.rotation());

    LeftSide->spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
    RightSide->spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);

    wait(20, vex::msec);

    this->mutex.unlock();

    return;
}

/**   
 * @brief Drive the Robot to a target point using distance formula
 * 
 * @param x x location to move to [inches]
 * @param y y location to move to [inches]
 * @param linearSettings The PID Settings to use for the movement
 * @param angularSettings The Angular PID Settings to use for the movement
 * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
 * @param reversed Allow the robot to do the motion backwards
 * @param async Determine whether or not to run command in a separate thread.
 * @param minSpeed Minimum speed to continue driving at for motion chaining. [default to 0]
 */
void Bucees::Robot::DriveToPoint(float x, float y, PIDSettings linearSettings, PIDSettings angularSettings, float timeout, bool reversed, bool async, float minSpeed) {

    if (async == true) {
        launch_task([&] {
            this->DriveToPoint(x, y, linearSettings, angularSettings, timeout, reversed, false);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    Linear->setGains(linearSettings);
    Angular->setGains(angularSettings);
    Linear->setTimeoutTime(timeout);

    Bucees::Coordinates initialCoordinates = this->getRobotCoordinates(true, reversed); // get the coordinates before the movement started
    Bucees::Coordinates targetCoordinates = reversed ? Bucees::Coordinates(-x, -y) : Bucees::Coordinates(x, y); // initalize the target coordinates using the coordinates class
    bool close = false; // whether or not the robot is close to the target point for settling

    float previousLinear;

    while (1) {
        Bucees::Coordinates currentCoordinates = this->getRobotCoordinates(true, reversed); // Get the current coordinates
        
        float targetTheta = reversed ? currentCoordinates.angle(targetCoordinates) + M_PI : currentCoordinates.angle(targetCoordinates); // Get the angle from the robot to the point
        //printf("targetTheta: %f \n", targetTheta);

        distanceTraveled = initialCoordinates.distance(this->getRobotCoordinates(true, false));

        float linearError = currentCoordinates.distance(targetCoordinates); // Get the distance from current to target in inches
        float angularError = remainderf(targetTheta - currentCoordinates.theta, M_PI); // Get the distance from the target theta to the current theta

        // Apply scales:
        linearError = reversed ? linearError * -cosf(angularError) : linearError * cosf(angularError);
        angularError = to_deg(angularError);
        //printf("lError: %f, aError: %f \n", linearError, angularError);

        // Calculate motor powers using PID:
        float linearMotorPower = Linear->calculateMotorPower(linearError);
        float angularMotorPower = close ? 0 : Angular->calculateMotorPower(angularError);

        const float radius = 1 / fabs(findCurvature(currentCoordinates, targetCoordinates, currentCoordinates.theta));
        const float maxSlipSpeed = sqrtf(8 * radius * 9.8);

        if (linearMotorPower > maxSlipSpeed) linearMotorPower = maxSlipSpeed;
        if (linearMotorPower < -maxSlipSpeed) linearMotorPower = -maxSlipSpeed;

        // Settling Condition:
        if (fabs(linearError) < 7.5) {
            close = true;
            //Linear->settings.kA = 2; // start deaccelearting
        }

        linearMotorPower = close ? slew(linearMotorPower, previousLinear, 1.5) : linearMotorPower;

        if (this->defaultMinSpeed != 0 && this->defaultMinSpeed > linearMotorPower && reversed != true) {
            linearMotorPower = this->defaultMinSpeed;
        } else if (this->defaultMinSpeed != 0 && -this->defaultMinSpeed < linearMotorPower && reversed == true) {
            linearMotorPower = -this->defaultMinSpeed;
        }

       // printf("x: %f, y: %f, theta: %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
        //printf("lMP: %f, aMP: %f \n", linearMotorPower, angularMotorPower);

        //printf("linearError:%f \n", linearError);

        // Apply motor powers:
        LeftSide->spin(vex::directionType::fwd, linearMotorPower + angularMotorPower, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, linearMotorPower - angularMotorPower, vex::voltageUnits::volt);

        previousLinear = linearMotorPower;

        if (fabs(linearError) <= 3.5 || Linear->isSettled()) break;

        wait(10, vex::msec);
    }

    Linear->reset();
    Angular->reset();

   if (this->defaultMinSpeed == 0) {
    LeftSide->stop(vex::brakeType::coast);
    RightSide->stop(vex::brakeType::coast);
   } else if (this->defaultMinSpeed != 0 && this->reversedChaining == false) {
    LeftSide->spin(vex::directionType::fwd, this->defaultMinSpeed, vex::voltageUnits::volt);
    RightSide->spin(vex::directionType::fwd, this->defaultMinSpeed, vex::voltageUnits::volt);
   } else if (this->defaultMinSpeed != 0 && this->reversedChaining == true) {
    LeftSide->spin(vex::directionType::fwd, -this->defaultMinSpeed, vex::voltageUnits::volt);
    RightSide->spin(vex::directionType::fwd, -this->defaultMinSpeed, vex::voltageUnits::volt);
   }

    distanceTraveled = -1;

    this->mutex.unlock();
}

/**
 * @brief Turn the Robot towards a target point using simple trigonometry
 * 
 * @param x x coordinate to face
 * @param y y coordinate to face
 * @param settings The PID Settings to use for the movement
 * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
 * @param async Determine whether or not to run command in a separate thread.
*/
void Bucees::Robot::TurnToPoint(float x, float y, PIDSettings settings, float timeout, bool async) {

    if (async == true) {
        launch_task([&] {
            this->TurnToPoint(x, y, settings, timeout, false);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    Angular->setGains(settings);
    Angular->setTimeoutTime(timeout);

    Coordinates targetCoordinates = Coordinates(x, y);

    float initalTheta = this->getRobotCoordinates().theta;
    float initTargetTheta = fmodf(to_deg(this->getRobotCoordinates().angle(targetCoordinates) + M_2_PI), 360);

    while (1) {
        Coordinates currentCoordinates = this->getRobotCoordinates(false);

        float targetTheta = fmodf(to_deg(currentCoordinates.angle(targetCoordinates) + M_2_PI), 360);

        float deltaTheta = remainderf(currentCoordinates.theta - targetTheta, 360); // scale the error to take the most efficient turn

        distanceTraveled = remainderf(currentCoordinates.theta - initalTheta, 360);

        //printf("deltaTheta: %f \n", deltaTheta);

        float motorPower = Angular->calculateMotorPower(0 - deltaTheta); // our goal is to reach 0 delta theta which is why we use 0 as our setpoint instead of doing target - current

        if (this->defaultMinSpeed != 0 && this->defaultMinSpeed > fabs(motorPower)) {
            motorPower = sgn(motorPower) * this->defaultMinSpeed;
        }

        LeftSide->spin(vex::directionType::fwd, motorPower, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, -motorPower, vex::voltageUnits::volt);

        if (Angular->isSettled() == true) break;

        wait(10, vex::msec);
    }

    Angular->reset();

    if (this->defaultMinSpeed == 0) {
        LeftSide->stop(vex::brakeType::hold);
        RightSide->stop(vex::brakeType::hold);
    } else if (this->defaultMinSpeed != 0 && sgn(initTargetTheta) == 1) { // motion chain to the right
        LeftSide->spin(vex::directionType::fwd, this->defaultMinSpeed, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, -this->defaultMinSpeed, vex::voltageUnits::volt);
    } else if (this->defaultMinSpeed != 0 && sgn(initTargetTheta) == -1) { // motion chain to the left
        LeftSide->spin(vex::directionType::fwd, -this->defaultMinSpeed, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, this->defaultMinSpeed, vex::voltageUnits::volt);
    }

    distanceTraveled = -1;

    this->mutex.unlock();
}

/**
 * @brief Move the Robot towards a target coordinate using a boomerang controller [DISABLED]
 * 
 * @param x x location to move to [inches]
 * @param y y location to move to [inches]
 * @param theta The angle to face at the end of the movement [degrees]
 * @param linearSettings The Linear PID Settings to use for the movement
 * @param angularSettings The Angular PID Settings to use for the movement
 * @param lead Determines how curved the robot will move
 * @param maxVoltages Determines the maximum speed overall of the motors [default to 12 voltages]
 * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
 * @param reversed Allow the robot to do the motion backwards
 * @param async Determine whether or not to run command in a separate thread. 
 * @param minSpeed Minimum speed to continue driving at for motion chaining. [default to 0]
 */
void Bucees::Robot::DriveToCoordinates(float x, float y, float theta, PIDSettings linearSettings, PIDSettings angularSettings, float lead, float maxVoltages, float timeout, bool reversed, bool async , float minSpeed) {

    if (async == true) {
        launch_task([&] {
            this->DriveToCoordinates(x, y, theta, linearSettings, angularSettings, lead, maxVoltages, timeout, reversed, false, minSpeed);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    float previousMaxVoltages = Linear->settings.maxVoltages;

    Linear->setGains(linearSettings);
    Angular->setGains(angularSettings);
    Linear->setTimeoutTime(timeout);
    Linear->setMaxVoltages(maxVoltages);
    Angular->setMaxVoltages(maxVoltages);

    Bucees::Coordinates initialCoordinates = this->getRobotCoordinates(); 

    Bucees::Coordinates target(x, y, to_rad(theta));

    bool previousSameSide = false;
    bool lateralSettled = false;
    bool close = false;

    while (1) {

        Bucees::Coordinates currentCoordinates = this->getRobotCoordinates(true, reversed);
        distanceTraveled = initialCoordinates.distance(currentCoordinates);

        const float distTarget = this->getRobotCoordinates(true, false).distance(target);

        Bucees::Coordinates carrotPoint = Bucees::Coordinates(target.x - distTarget * sinf(target.theta) * lead, target.y - distTarget * cosf(target.theta) * lead);
        
        const float targetTheta = reversed ? currentCoordinates.angle(carrotPoint) + M_PI : currentCoordinates.angle(carrotPoint);
        
        float linearError = currentCoordinates.distance(carrotPoint); // Get the distance from current to target in inches
        float angularError = remainderf(targetTheta - currentCoordinates.theta, M_PI); // Get the distance from the target theta to the current theta

        // Apply scales:
        linearError = reversed ? linearError * -cosf(angularError) : linearError * cosf(angularError);
        angularError = to_deg(angularError);

        // Calculate motor powers using PID:
        float linearMotorPower = close ? 0 : Linear->calculateMotorPower(linearError);
        float angularMotorPower = Angular->calculateMotorPower(angularError);

        //Slip speed logic:
        const float radius = 1 / fabs(findCurvature(currentCoordinates, carrotPoint, currentCoordinates.theta));
        const float maxSlipSpeed = sqrtf(8 * radius * 9.8);

        if (linearMotorPower > maxSlipSpeed) linearMotorPower = maxSlipSpeed;
        if (linearMotorPower < -maxSlipSpeed) linearMotorPower = -maxSlipSpeed;

        //prioritize angular motion
        const float overturn = fabs(angularMotorPower) + fabs(linearMotorPower) - maxVoltages;
        if (overturn > 0) linearMotorPower -= linearMotorPower > 0 ? overturn : -overturn;

        //Motion chaining conditions:
        if (this->defaultMinSpeed != 0 && this->defaultMinSpeed > linearMotorPower && reversed != true) {
            linearMotorPower = this->defaultMinSpeed;
        } else if (this->defaultMinSpeed != 0 && -this->defaultMinSpeed < linearMotorPower && reversed == true) {
            linearMotorPower = -this->defaultMinSpeed;
        }

        // Settling Condition:
        if (fabs(linearError) < 10) {
            close = true;
        };

        printf("lError: %f \n", linearError);

        //printf("distT: %f \n", distTarget);

        // printf("lP: %f, aP: %f \n", linearMotorPower, angularMotorPower);
        // printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
        
        // Apply motor powers:
        LeftSide->spin(vex::directionType::fwd, linearMotorPower + angularMotorPower, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, linearMotorPower - angularMotorPower, vex::voltageUnits::volt);

        if (fabs(linearError) <= 7.5 || Linear->isSettled() == true) break;

        wait(10, vex::msec);
    }

    Linear->setMaxVoltages(previousMaxVoltages);
    Linear->reset();

   if (this->defaultMinSpeed == 0) {
    LeftSide->stop(vex::brakeType::hold);
    RightSide->stop(vex::brakeType::hold);
   } else if (this->defaultMinSpeed != 0 && this->reversedChaining == false) {
    LeftSide->spin(vex::directionType::fwd, this->defaultMinSpeed, vex::voltageUnits::volt);
    RightSide->spin(vex::directionType::fwd, this->defaultMinSpeed, vex::voltageUnits::volt);
   } else if (this->defaultMinSpeed != 0 && this->reversedChaining == true) {
    LeftSide->spin(vex::directionType::fwd, -this->defaultMinSpeed, vex::voltageUnits::volt);
    RightSide->spin(vex::directionType::fwd, -this->defaultMinSpeed, vex::voltageUnits::volt);
   }

    distanceTraveled = -1;

    this->mutex.unlock();

}

void Bucees::Robot::activateMotionChaining(float linearMinSpeed, float angularMinSpeed) {
    this->linearMinSpeed = linearMinSpeed;
    this->angularMinSpeed = angularMinSpeed;
}

void Bucees::Robot::deactivateMotionChaining() {
    this->linearMinSpeed = 0;
    this->angularMinSpeed = 0;
}

/**
 * @brief Drive to a point using a ramsette controller, a more advanced version of drive to point using the BLRS wiki documentation [DISABLED]
 * 
 * @param x x location that was inputted
 * @param y y location that was inputted
 * @param theta angle in degrees that was inputted
 * @param beta A proportional term for the controller (0 < x)
 * @param zeta A damping term similar to derivative in PID (0 < x < 1)
 * @param linear Desired linear velocity (should come from a motion profile)
 * @param angular Desired angular velocity (should come from a motion profile)
 * @param async Determine whether or not to run command in a separate thread. 
*/
/*
void Bucees::Robot::ramseteDriveTo(float x, float y, float theta, float beta, float zeta, float linear, float angular, bool async) {

    if (async == true) {
        launch_task([&] {
            this->ramseteDriveTo(x, y, theta, beta, zeta, linear, angular, false);
        });
        wait(10, vex::msec);
        return;
    }

    this->mutex.lock();

    Coordinates initialCoordinates = this->getRobotCoordinates(true);
    Coordinates targetCoordinates(x, y, to_rad(theta));

    while (1) {

        Coordinates currentCoordinates = this->getRobotCoordinates(true);

        Coordinates errorCoordinates = targetCoordinates - currentCoordinates;
        errorCoordinates.theta = targetCoordinates.theta - currentCoordinates.theta; // operator "-" doesn't subtract theta

        // transform error coordinates to a eigen vector:
        Eigen::Vector3f errorCoordinatesVector;
        Eigen::Matrix3f transformationMatrix;

        // initalize transformation matrix based off of BLRS documentation:
        transformationMatrix << cosf(currentCoordinates.theta), sinf(currentCoordinates.theta),  0,
                                -sinf(currentCoordinates.theta), cosf(currentCoordinates.theta), 0,
                                0,                               0,                              1;

        errorCoordinatesVector << errorCoordinates.x, errorCoordinates.y, errorCoordinates.theta;

        Eigen::Vector3f localErrorMatrix = transformationMatrix * errorCoordinatesVector; // apply matrix * vector

        float kGain = 2 * zeta * sqrtf(powf(angular, 2) + beta * powf(linear, 2)); // calculate the k gain

        float linearVelocity = linear * cosf(localErrorMatrix(2)) + kGain * localErrorMatrix(0);
        float angularVelocity = angular + kGain * localErrorMatrix(2) + (beta * linear * sinf(localErrorMatrix(2)) * localErrorMatrix(1)) / localErrorMatrix(2);

        float linearMotorVelocity = linearVelocity / (drivetrainWheelDiameter * M_1_PI);
        
        float leftMotorVelocity = linearMotorVelocity + angularVelocity;
        float rightMotorVelocity = linearMotorVelocity - angularVelocity;

        LeftSide->spin(vex::directionType::fwd, leftMotorVelocity, vex::voltageUnits::volt);
        RightSide->spin(vex::directionType::fwd, rightMotorVelocity, vex::voltageUnits::volt);

        if (currentCoordinates.distance(targetCoordinates) < 1) break;
        
        distanceTraveled = initialCoordinates.distance(currentCoordinates);

        wait(10, vex::msec);
    }

    LeftSide->stop(vex::brakeType::coast);
    RightSide->stop(vex::brakeType::coast);
    distanceTraveled = -1;

    this->mutex.unlock();

}
*/