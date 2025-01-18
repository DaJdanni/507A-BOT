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

//vex::aivision visionSensor(PORT)

inertial InertialSensor(PORT6);

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
  0.45, 
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

  0, // 5

  1.f/1.f
);

// BACK TRACKER EXAMPLE:
Bucees::TrackingWheel BackTracker(
  PORT1,

  false,

  2.75,
  
  -3, // -3

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
bool toggleAlignmentSystem = false;
bool allowLBRotation = true;
bool inAlignment = false;
bool ladyBrownMacro = false;
bool inStageMacro = false;

// lady brown macro
const int lBStages = 2; // the amount of stages
const int timeOutTime = 750; // change how long it has to reach the target
const int lBMotorPower = 12; // change the maximum speed
const int stopperDegrees = 440; // where to stop lb
int currentStage = -1;
int targetStage = 0;
double stages[lBStages] = { // the stages and their degrees
  67,
  105,
};

void lBPid(double target) {

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

    printf("Rotation Position: %f \n", rotationPosition);

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

void togglelBF() {
  if (togglelBDB == true) return;
  inStageMacro = true;
  togglelBDB = true;

  currentStage += 1;
  if (currentStage == lBStages) currentStage = 0;
  targetStage = stages[currentStage];

  std::cout << "Toggle lB" << std::endl;

  launch_task([&] {lBPid(targetStage);}); // start pid

  wait(200, msec);
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

  Robot.initOdom();
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

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  // launch_task([&] {
  //   lBPid(67);
  //   wait(200, msec);
  //   lBPid(190);
  // });

  Intake.spin(reverse, 12, volt);

  Doinker.open();
  Robot.DriveToPoint(0, 38, L_Settings, A60_Settings, 0, false, true);

  wait(550, msec);

  Linear.setMaxVoltages(10.5);

  wait(375, msec);

  goalRush.open();

  Robot.waitChassis();

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 14.5, L_Settings, A0_Settings, 800, true, true);

  wait(100, msec);
  Intake.stop();
  wait(550, msec);;
  goalRush.close();
  wait(100, msec);
  Doinker.close();
  wait(500, msec);

  Robot.waitChassis();

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(7.5);

  Robot.DriveToPoint(-6, 42.5, L_Settings, A0_Settings, 2000, true);

  Linear.setMaxVoltages(12);

  Clamp.open();

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(10);

  Robot.DriveToPoint(0, 2.5, L_Settings, A0_Settings);

  wait(350, msec);

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(20, 15, L_Settings, A0_Settings, 0, true);

  Robot.TurnToPoint(20, 30, A120_Settings, 800);

  Clamp.close();

  Robot.TurnFor(180, A120_Settings, 800);

  printCoordinates();

  Robot.DriveToPoint(26, 39, L_Settings, A0_Settings, 0, true);

  Intake.stop();

  Clamp.open();

  printCoordinates();

  Robot.TurnFor(240, A60_Settings, 500);

  Linear.setMaxVoltages(12);

  // activateMotionChaining(false, 5);

  // Robot.DriveToPoint(5, 30, L_Settings, A0_Settings);

  // deactivateMotionChaining();

  Intake.spin(reverse, 12, volt);

  Robot.DriveToCoordinates(0, -2, 200, L_Settings, A0_Settings, 0.7, 12, 1250);

  Robot.DriveFor(36, L_Settings);

  //Robot.DriveToPoint(-4.5, -10, L_Settings, A0_Settings, 2000);

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

    if (rotationPosition > stopperDegrees) {
      inAlignment = true;
      //ladyBrown.stop(hold);
    } else {
      inAlignment = false;
    }

    wait(10, msec);
  }
}

void checkAlignment() {
  if (inAlignment == true) {
    allowLBRotation = true;
  } else {
    allowLBRotation = false;
  }
}

void toggleAlignmentSystemF() {
  toggleAlignmentSystem = !toggleAlignmentSystem;
}

float lBSpeedReverse = 12;

void usercontrol(void) {

  Controller.ButtonB.pressed(toggleClampF);
  Controller.ButtonY.pressed(toggleDoinkerF);
  Controller.ButtonRight.pressed(toggleGoalRushF);
  Controller.ButtonL1.pressed(togglelBF);
  Controller.ButtonR1.pressed(checkAlignment);
  Controller.ButtonR1.pressed(toggleStageMacro);
  Controller.ButtonDown.pressed(toggleAlignmentSystemF);

  launch_task([&] {
    detectAlignment();
  });
  
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
      Intake.spin(forward, 12, volt);
    } else if (Controller.ButtonR2.pressing()) {
      Intake.spin(reverse, 12, volt);
    } else {
      Intake.stop();
    }

    float rotationPosition = (ladyBrown1.position(degrees) + ladyBrown2.position(degrees)) / 2;

    if (Controller.ButtonR1.pressing()) {
      ladyBrown.spin(forward, 12, volt);
    } else if (rotationPosition < 1) {
      ladyBrown.stop(hold);
    } else if (ladyBrownMacro == false && inStageMacro == false && rotationPosition > 1) {
      std::cout << "spinning " << std::endl;
      ladyBrown.spin(reverse, 5, volt);
    }

    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
   // std::cout << "Color: " << RingFilter.isNearObject() << std::endl;
    printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);


    //printf("Rotation Position: %f \n", rotationPosition);

    Brain.Screen.printAt(50, 25, "BackLeft Temp: %f", BottomLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 50, "BackRight Temp: %f", BottomRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 75, "TopLeft Temp: %f", TopLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 100, "TopRight Temp: %f", TopRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 125, "FrontLeft Temp: %f", FrontLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 150, "FrontRight Temp: %f", FrontRight.temperature(temperatureUnits::fahrenheit));

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
