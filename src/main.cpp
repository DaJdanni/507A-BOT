/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jkeep                                                     */
/*    Created:      1/1/2025, 7:18:58 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
vex::brain Brain = vex::brain();
vex::controller Controller = vex::controller();

motor FrontLeft = motor(PORT17, ratio6_1, false); //false 17
motor BottomLeft = motor(PORT18, ratio6_1, true); //true 18
motor TopLeft = motor(PORT20, ratio6_1, true); //true 20

motor FrontRight = motor(PORT15, ratio6_1, true); //true 15
motor BottomRight = motor(PORT16, ratio6_1, false);//false 16
motor TopRight = motor(PORT19, ratio6_1, false);//false 19

motor Intake = motor(PORT10, ratio6_1, false);

motor ladyBrown1 = motor(PORT11, false);
motor ladyBrown2 = motor(PORT12, true);

motor_group LeftSide = motor_group(FrontLeft, BottomLeft, TopLeft);
motor_group RightSide = motor_group(FrontRight, BottomRight, TopRight);
motor_group ladyBrown = motor_group(ladyBrown1, ladyBrown2);

rotation frontTracker(PORT3, true);
rotation backTracker(PORT8); 

optical RingFilter(PORT4);
optical RingFilterBottom(PORT7);

distance GoalDetector(PORT6);

//vex::aivision visionSensor(PORT)

inertial InertialSensor(PORT5);

Bucees::PIDSettings LOdom_Settings {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  5,
  // PROPORTIONAL GAIN
  1.3, 
  // INTEGRAL GAIN
  0.0, 
  // DERIVATIVE GAIN
  0.7,  //4.5, 4.7, 5.2, 
  // EXIT ERROR
  2.5, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12  
};

Bucees::PIDSettings L_Settings {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  5,
  // PROPORTIONAL GAIN
  1.1275, 
  // INTEGRAL GAIN
  0.0, 
  // DERIVATIVE GAIN
  5.2,  //4.5, 4.7, 5.2, 
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12  
};

Bucees::PIDSettings LMogo_Settings { // for 6 rings
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  5,
  // PROPORTIONAL GAIN
  1.1275, 
  // INTEGRAL GAIN
  0.0, 
  // DERIVATIVE GAIN
  5.2,  //4.5, 4.7, 5.2, 
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12  
};

Bucees::PIDSettings A0_Settings { // not done [0-60]
// right turn: good, left turn: bad 
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.32,
  // INTEGRAL GAIN
  0.0137,
  // DERIVATIVE GAIN
  2.1,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings A60_Settings { // DONE [60-120]
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.32,
  // INTEGRAL GAIN
  0.00825,
  // DERIVATIVE GAIN
  2.1,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings A120_Settings { // DONE [120-180]
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  0,
  // PROPORTIONAL GAIN
  0.29, 
  // INTEGRAL GAIN
  0.0088125, // 0.00881 for 180,
  // DERIVATIVE GAIN
  2.1, 
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  17.5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings ANTIDRIFT_1TILE {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.125,
  // INTEGRAL GAIN
  0.0,
  // DERIVATIVE GAIN
  0.0,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings FishSettings {
   // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.125,
  // INTEGRAL GAIN
  0.0,
  // DERIVATIVE GAIN
  0.25,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12 
};

Bucees::PIDSettings ladyBrownSettings {
  // FEEDFORWARD GAIN
  0.0, 
  // ACCELERATION GAIN 
  0.0,
  // PROPORTIONAL GAIN
  0.25, 
  // INTEGRAL GAIN
  0.03,
  // DERIVATIVE GAIN
  0.465, 
  // EXIT ERROR
  0.5, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12  
};

Bucees::PIDSettings ladyBrownSettings2 {
  // FEEDFORWARD GAIN
  0.0, 
  // ACCELERATION GAIN 
  0.0,
  // PROPORTIONAL GAIN
  0.24, 
  // INTEGRAL GAIN
  0.031,
  // DERIVATIVE GAIN
  0.11, 
  // EXIT ERROR
  0.5, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12  
};

Bucees::FAPIDController Linear(L_Settings);
Bucees::FAPIDController Angular(A60_Settings);
Bucees::FAPIDController AntiDrift(A0_Settings);
Bucees::FAPIDController lBController(ladyBrownSettings);

Bucees::TrackingWheel RightTracker(
  PORT14,

  true,

  2,

  -0.5, // 5

  1.f/1.f
);

// BACK TRACKER EXAMPLE:
Bucees::TrackingWheel BackTracker(
  PORT8,

  false,

  2.75,
  
  -2.5, // -3

  1.f/1.f
);

Bucees::Robot Robot(
  // INSERT THE WHEEL DIAMETER OF YOUR DRIVETRAIN WHEELS:
  Bucees::Omniwheel::NEW_325,

  // INSERT THE GEAR RATIO OF YOUR DRIVETRAIN USING: X.f/X.f (REPLACE THE X WITH YOUR DRIVETRAIN GEAR RATIO):
  36.f/48.f,
  
  // INSERT THE TRACK WIDTH OF YOUR DRIVETRAIN IN INCHES:
  11.412,

  // INSERT THE LEFT MOTOR GROUP OF YOUR DRIVETRAIN:
  &LeftSide,

  // INSERT THE RIGHT MOTOR GROUP OF YOUR DRIVETRAIN:
  &RightSide,

  // INSERT THE PORT OF YOUR INERTIAL SENSOR, IF YOU DO NOT USE AN INERTIAL SENSOR, REPLACE WITH "nullptr"
  PORT5,

  // INSERT THE RIGHT TRACKING WHEEL OBJECT:
  &RightTracker,

  // INSERT THE BACK TRACKING WHEEL OBJECT [REPLACE WITH NULLPTR IF NO BACK TRACKER]:
  &BackTracker,

  // INSERT THE LINEAR PID SETTINGS OBJECT:
  &Linear,

  // INSERT THE ANGULAR PID SETTINGS OBJECT:
  &Angular,

  // INSERT THE ANTI DRIFT PID SETTINGS OBJECT:
  &AntiDrift
);

// for macros:
template <class F>
vex::task launch_task(F&& function) {
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

// toggles:
bool togglelBDB;
bool inStageDB;
bool toggleClampDB;
bool toggleClamp;
bool toggleDoinkerDB;
bool toggleDoinker;
bool toggleGoalRushDB;
bool toggleGoalRush;
bool toggleAlignment = false;
bool allowForSpin = false;
bool startReset = false;
bool inAlignment = false;
bool ladyBrownMacro = false;
bool inStageMacro = false;

// lady brown macro
const int lBStages = 2; // the amount of stages
const int timeOutTime = 800; // change how long it has to reach the target
const int lBMotorPower = 12; // change the maximum speed
const int stopperDegrees = 440; // where to stop lb
int currentStage = -1;
int targetStage = 0;
double stages[lBStages] = { // the stages and their degrees
  80,
  105,
};

void lBPid(double target, double defaultTimeout, double defaultSpeed) {

  lBController.setTimeoutTime(defaultTimeout);

  std::cout << "currentStage: " << currentStage << std::endl;
  std::cout << "targetStage: " << target << std::endl; 

  if (currentStage == 0) {
    lBController.setGains(ladyBrownSettings);
  } else {
    lBController.setGains(ladyBrownSettings2);
  }

  ladyBrownMacro = true;
  inStageMacro = true;

  while (1) {

    float rotationPosition = (ladyBrown1.position(degrees) + ladyBrown2.position(degrees)) / 2;

    float motorPower = lBController.calculateMotorPower(target - rotationPosition);

    //std::cout << "rotPosition: " << rotationPosition << std::endl;
    //std::cout << "error: " << target - rotationPosition << std::endl;

    //printf("Rotation Position: %f \n", rotationPosition);

    inStageMacro = true;

    ladyBrown.spin(fwd, motorPower, volt);

    if (lBController.isSettled() == true) break;

    wait(10, msec);
  }

  lBController.reset();
  ladyBrown.stop(hold);

  float rotationPosition = ladyBrown.position(degrees);

  std::cout << "Finished LB PID" << std::endl;
  std::cout << "Final Position: " << rotationPosition << std::endl;

  ladyBrownMacro = false;
}

void toggleAlignmentF() {
  if (togglelBDB == true) return;
  inStageMacro = true;
  togglelBDB = true;

  toggleAlignment = !toggleAlignment;

  currentStage = -1;

  launch_task([&] {lBPid(stopperDegrees, 1250, 12);});

  waitUntil(ladyBrownMacro == false);
  togglelBDB = false;
}

void togglelBF() {
  if (togglelBDB == true) return;
  inStageMacro = true;
  togglelBDB = true;

  currentStage += 1;
  if (currentStage == lBStages) currentStage = 0;
  targetStage = stages[currentStage];

  std::cout << "Toggle lB" << std::endl;

  launch_task([&] {lBPid(targetStage);}); // start pid

  waitUntil(ladyBrownMacro == false);
  togglelBDB = false;
}

void toggleStageMacro() {
  if (inStageDB == true) return;
  
  inStageMacro = false;
  currentStage = -1;

  wait(50, msec);
  inStageDB = false;
}

void toggleClampF() {
  if (toggleClampDB == true) return;
  toggleClampDB = true;

  pneumatics Clamp(Brain.ThreeWirePort.A);

  toggleClamp = !toggleClamp;

  if (toggleClamp == true) {
    Clamp.open();
  } else {
    Clamp.close();
  }


  wait(50, msec);
  toggleClampDB  = false;
}

void toggleDoinkerF() {
  if (toggleDoinkerDB == true) return;
  toggleDoinkerDB = true;

  pneumatics Doinker(Brain.ThreeWirePort.B);

  toggleDoinker = !toggleDoinker;

  if (toggleDoinker == true) {
    Doinker.open();
  } else {
    Doinker.close();
  }


  wait(50, msec);
  toggleDoinkerDB  = false;
}

void toggleGoalRushF() {
  if (toggleGoalRushDB == true) return;
  toggleGoalRushDB = true;

  pneumatics goalRush(Brain.ThreeWirePort.C);

  toggleGoalRush = !toggleGoalRush;

  if (toggleGoalRush == true) {
    goalRush.open();
  } else {
    goalRush.close();
  }


  wait(50, msec);
  toggleGoalRushDB  = false;
}

int targetIntakeSpeed = 0;
bool isJammed = false;
bool detectJams = true;

void rawSetIntake(int input) {
  Intake.spin(reverse, input, volt);
}

void set_intake(int input) {
  rawSetIntake(input);
  targetIntakeSpeed = input;
}

void intakeAntiJam() {
  const int waitTime = 30;
  const int outtakeTime = 250;
  const int minSpeed = 2;
  
  int jamCounter = 0;

  while (true) {
    // anti jam logic:
    if (isJammed == true && detectJams == true) {
      //std::cout << "is jammed" << std::endl;
      std::cout << jamCounter << std::endl;
      rawSetIntake(-12 * sgn(targetIntakeSpeed));
      jamCounter += 10;

      if (jamCounter > outtakeTime) {
        isJammed = false;
        jamCounter = 0;
        rawSetIntake(targetIntakeSpeed);
      }
    }
    // Detect a jam if we are trying to intake but our motor reads a velocity of 0
    else if (abs(targetIntakeSpeed) >= minSpeed && Intake.velocity(pct) == 0) {
      jamCounter += 10;

      if (jamCounter > waitTime) {
        jamCounter = 0;
        isJammed = true;
      }
    }

    if (targetIntakeSpeed <= minSpeed) {
      jamCounter = 0;
    }

    wait(10, msec);
  }
}


void pre_auton(void) {
  FrontLeft.setBrake(coast);
  TopLeft.setBrake(coast);
  BottomLeft.setBrake(coast);
  FrontRight.setBrake(coast);
  TopLeft.setBrake(coast);
  BottomRight.setBrake(coast);

  Intake.setBrake(coast);
  ladyBrown1.setBrake(hold);
  ladyBrown2.setBrake(hold);

  LeftSide.resetPosition();
  RightSide.resetPosition();
  ladyBrown.resetPosition();

  lBController.setTimeoutTime(timeOutTime);
  lBController.setMaxVoltages(lBMotorPower);

  InertialSensor.calibrate();
  waitUntil(InertialSensor.isCalibrating() == false);

  RingFilter.setLightPower(100, pct);
  RingFilterBottom.setLightPower(100, pct);

  RingFilter.setLight(ledState::on);
  RingFilterBottom.setLight(ledState::on);

  Robot.initOdom();

  launch_task([&] {intakeAntiJam();});
}

void activateMotionChaining(bool reversed, float minSpeed) {
  L_Settings.exitError = 8;
  A0_Settings.exitError = 20;
  A60_Settings.exitError = 20;
  A120_Settings.exitError = 20;
  Linear.setExitError(8);
  Angular.setExitError(20);
  Robot.defaultMinSpeed = minSpeed;
  Robot.reversedChaining = reversed;
}

void deactivateMotionChaining(bool reversed) {
  L_Settings.exitError = 1;
  A0_Settings.exitError = 1;
  A60_Settings.exitError = 1;
  A120_Settings.exitError = 1;
  Linear.setExitError(1);
  Angular.setExitError(1);
  Robot.defaultMinSpeed = 0;
  LeftSide.stop(hold);
  RightSide.stop(hold);
  Robot.reversedChaining = reversed;
}

void printCoordinates(bool reversed) {
    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false, reversed);
    printf("ATM right now: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
}


void autonomous(void) {
 // fiveRingAllianceStake(false);

 negSideBlue(false);
}

double driveCurve(double x, double scale) {
  if (scale != 0) {
    return (scale * pow(1.01, x) - scale);
    //return (pow(2.718, (scale * (std::fabs(x) - 12))) / 1000) * x;
  }
  return x;
}

void detectAlignment() {
  while (1) {

    float rotationPosition = (ladyBrown1.position(degrees) + ladyBrown2.position(degrees)) / 2;

    if (rotationPosition < 1 && startReset == true) {
      startReset = false;
    }

    if (rotationPosition > stopperDegrees) {
      inAlignment = true;
      //ladyBrown.stop(hold);
    } else {
      inAlignment = false;
    }

    wait(10, msec);
  }
}

void checkAlignment2() {
  if (inAlignment == true && allowForSpin == true) {
    std::cout << "stop this madness" << std::endl;
    allowForSpin = false;
    startReset = true;
  }
  if (inAlignment == true) {
    std::cout << "hi" << std::endl;
    allowForSpin = true;
  }
}

float lBSpeedReverse = 12;
pneumatics goalRush(Brain.ThreeWirePort.C);

int detections[2] = {0, 0};
int objectsDetected = 0;

void filterTest() {
  // if (RingFilter.hue() > 150 && redFilter == true) {return;}
  // if (RingFilter.hue() < 15 && blueFilter == true) {return;}
  //if (RingFilter.hue() < 15) std::cout << "RED" << std::endl;
 // if (RingFilter.hue() > 150) std::cout << "BLUE" << std::endl;

  if (RingFilter.hue() < 150 && RingFilter.isNearObject()) {
    std::cout << "HEY NO RED RINGS" << std::endl;
    wait(120, msec);
    Intake.spin(fwd, 12, volt);
    wait(50, msec);
    Intake.spin(reverse, 12, volt);
  }
}

void usercontrol(void) {

  Controller.ButtonB.pressed(toggleClampF);
  Controller.ButtonY.pressed(toggleDoinkerF);
  Controller.ButtonRight.pressed(toggleGoalRushF);
  Controller.ButtonL1.pressed(togglelBF);
  //Controller.ButtonR1.pressed(checkAlignment);
  Controller.ButtonR1.pressed(toggleStageMacro);
 //Controller.ButtonR1.released(checkAlignment2);
  Controller.ButtonDown.pressed(toggleAlignmentF);

  //Intake.spin(reverse, 12, volt);

  launch_task([&] {
    detectAlignment();
  });

  // launch_task([&] {
  //   while (1) {
  //         filterTest();
  //     wait(10, msec);
  //   };
  // });

  while (1) {

    Brain.Screen.clearScreen();

    double LeftJoystickPosition = Controller.Axis3.position();
    double RightJoystickPosition = Controller.Axis1.position();

    if (fabs(LeftJoystickPosition) < 5) {
      LeftJoystickPosition = 0;
    }

    if (fabs(RightJoystickPosition) < 5) {
      RightJoystickPosition = 0;
    }

   // LeftJoystickPosition = driveCurve(LeftJoystickPosition, 18.1212);
    RightJoystickPosition = driveCurve(RightJoystickPosition, 18.1212);

    LeftJoystickPosition *= 0.12;
   // RightJoystickPosition *= 0.12;

  //  printf("lJP: %f \n", LeftJoystickPosition);
  //  printf("rJP: %f \n", RightJoystickPosition);

  // std::cout << "currentStage: " << currentStage << std::endl;
  // std::cout << "targetStage: " << targetStage << std::endl;

    LeftSide.spin(fwd, LeftJoystickPosition + RightJoystickPosition, volt);
    RightSide.spin(fwd, LeftJoystickPosition - RightJoystickPosition, volt);

    //lBSpeedReverse = toggleAlignmentSystem ? 3 : 12;

    // intakeSpeed = intakeDetectMode ? 11 : 12;
    // reverseIntakeSpeed = intakeDetectMode ? 6 : 12;

    if (Controller.ButtonL2.pressing()) {
     Intake.spin(fwd, 12, volt);
     //set_intake(-12);
    } else if (Controller.ButtonR2.pressing()) {
      Intake.spin(reverse, 12, volt);
      //set_intake(12);
    } else {
      //set_intake(0);
      Intake.stop(coast);
    }

    float rotationPosition = (ladyBrown1.position(degrees) + ladyBrown2.position(degrees)) / 2;

    if (Controller.ButtonR1.pressing()) {
      ladyBrown.spin(forward, 12, volt);
    } else if (rotationPosition < 1) {
      ladyBrown.stop(hold);
    } else if ((Controller.ButtonR1.pressing() == false && ladyBrownMacro == false && inStageMacro == false && rotationPosition > 1)) {
      std::cout << "spinning " << std::endl;
      ladyBrown.spin(reverse, 5, volt);
    }

    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);

   // std::cout << "Color: " << RingFilter.isNearObject() << std::endl;
    //printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);

   // std::cout << "distance: " << GoalDetector.objectDistance(inches) << std::endl;
   // std::cout << "raw size: " << GoalDetector.objectRawSize() << std::endl;

    //std::cout << "stage: " << currentStage << std::endl;

    Brain.Screen.printAt(50, 25, "BackLeft Temp: %f", BottomLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 50, "BackRight Temp: %f", BottomRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 75, "TopLeft Temp: %f", TopLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 100, "TopRight Temp: %f", TopRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 125, "FrontLeft Temp: %f", FrontLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 150, "FrontRight Temp: %f", FrontRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 175, "Intake Temp: %f", Intake.temperature(temperatureUnits::fahrenheit));

    Brain.Screen.drawImageFromFile("Brain_Screen_Logo.png", 0, 0);

    Brain.Screen.render();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
