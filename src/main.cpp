/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       JDANNI                                                    */
/*    Created:      12/14/2023, 9:35:31 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

//13,14,15

// define your global instances of motors and other devices here
vex::brain Brain = vex::brain();
vex::controller Controller = vex::controller();

motor FrontLeft(PORT1, gearSetting::ratio6_1, true);
motor TopLeft(PORT2, gearSetting::ratio6_1, false);
motor BackLeft(PORT3, gearSetting::ratio6_1, true);

motor FrontRight(PORT6, gearSetting::ratio6_1, false);
motor TopRight(PORT7, gearSetting::ratio6_1, true);
motor BackRight(PORT8, gearSetting::ratio6_1, false);

motor Intake(PORT9);

motor FishMech(PORT18, gearSetting::ratio36_1, true);

inertial InertialSensor(PORT12);

motor_group LeftSide(FrontLeft, TopLeft, BackLeft);
motor_group RightSide(FrontRight, TopRight, BackRight);
drivetrain Drivetrain(LeftSide, RightSide);

rotation backTrackerSensor(PORT19);
//rotation rightTrackerSensor(PORT20, false);
rotation fishMechSensor(PORT15);

optical RingFilter(PORT14);

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

Bucees::FAPIDController Linear(L_Settings);

Bucees::FAPIDController Angular(A60_Settings);

Bucees::FAPIDController AntiDrift(A0_Settings);

Bucees::FAPIDController FishController(FishSettings);

//Bucees::FAPIDController LiftController(LIFT_SETTINGS);

// TRACKING WHEEL EXAMPLE:
/*Bucees::TrackingWheel RightTracker(
  // INSERT RIGHT TRACKER MOTOR GROUP IF NO TRACKING WHEEL, INSERT ROTATION SENSOR PORT IF TRACKING WHEEL:
  PORT16,

  // UNCOMMENT AND PUT REVERSED = TRUE/FALSE DEPENDING ON SETUP
  false,

  // INSERT WHEEL DIAMETER FOR THE TRACKING WHEEL [OR WHEEL DIAMETER FOR THE MOTOR GROUP]:
  2.9,

  // INSERT THE OFFSET FROM THE TRACKING CENTER:
  0,

  // INSERT THE GEAR RATIO OF THE TRACKING WHEEL [OR GEAR RATIO OF THE MOTOR GROUP]:
  1.f/1.f
);*/

// RIGHT TRACKER DRIVETRAIN MOTORS EXAMPLE:
Bucees::TrackingWheel RightTracker(
  &RightSide,

  3.25,

  5, // 5

  36.f/48.f
);

// BACK TRACKER EXAMPLE:
Bucees::TrackingWheel BackTracker(
  PORT11,

  false,

  -2.75,
  
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
  PORT12,

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

bool mogoDB;
bool toggleMogo;
bool intakeLiftDB;
bool toggleIntakeLift;
bool liftClampDB;
bool toggleLiftClamp;
bool liftRatchetDB;
bool toggleLiftRatchet;
bool extenderDB;
bool toggleExtender;
bool startMatchDB;
bool toggleStartMatch;

bool detectModeDB;
bool intakeDetectMode;
bool toggleResetFish;

bool inLiftMacro = false;
bool cancelMacro = false;

void halfOpen(double desiredPosition = 125) {
  // Lift.stop();
  // Lift.spin(fwd, 12, volt);
  // waitUntil(Lift.position(degrees) > desiredPosition);
  // Lift.stop();
}

void resetLift() {
  // Lift.stop();
  // Lift.spin(reverse, 4.5, volt);
  // waitUntil(Lift.position(degrees) < 0);
  // Lift.stop();
}


void toggleStartMatchF() {

  pneumatics LiftClamp(Brain.ThreeWirePort.C);

  if (startMatchDB == true) {return;}
  startMatchDB = true;
  toggleStartMatch = true;

  printf("hi chode strart \n");

  halfOpen(60);
  LiftClamp.open();
  wait(250, msec);
  LiftClamp.close();
  resetLift();
  // Lift.spin(reverse, 6, volt);
  // wait(0.2, seconds);
  // Lift.stop();

  wait(50, msec);
  toggleStartMatch = false;
  startMatchDB = false;
}


void toggleMogoF() {

  pneumatics Mogo(Brain.ThreeWirePort.G);

  printf("PRessed mOgo\n");

  if (mogoDB == true) {return;}
  mogoDB = true;

  toggleMogo = !toggleMogo;

  if (toggleMogo == true) {
    Mogo.open();
  } else {
    Mogo.close();
  }

  wait(50, msec);
  mogoDB = false;
}

void toggleIntakeLiftF() {

  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  if (intakeLiftDB == true) return;
  intakeLiftDB = true;

  toggleIntakeLift = !toggleIntakeLift;

  if (toggleIntakeLift == true) {
    IntakeLift.open();
  } else {
    IntakeLift.close();
  }

  wait(50, msec);
  intakeLiftDB = false;
}

const float liftDesiredPosition = 272; // degrees
const float fishDesiredPosition = 5; // degrees

void fishMechLoop(double desiredPosition = fishDesiredPosition, directionType dir = reverse) {
  
  FishController.setTimeoutTime(800);

  while (1) {

    float rotationPosition = fishMechSensor.position(degrees);

    float motorPower = FishController.calculateMotorPower(desiredPosition - rotationPosition);

  //  printf("mp: %f\n", motorPower);

    FishMech.spin(dir, motorPower, volt);

  //  printf("rotatoinPosition:%f\n", rotationPosition);

    if (FishController.isSettled() == true) break;
    if (cancelMacro == true) break;
    
    wait(10, msec);
  }

  std::cout << "ended" << std::endl;


  FishController.reset();
  FishMech.stop();
}

void resetFishMech() {

  if (inLiftMacro == true) return;
  cancelMacro = false;
  inLiftMacro = true;

  fishMechLoop();

  // FishMech.spin(fwd, 10, volt);
  // std::cout << "fishchigga: " << fishMechSensor.position(degrees) << std::endl;
  // waitUntil(fishDesiredPosition - fishMechSensor.position(degrees) < 6.5 || cancelMacro == true);

  if (cancelMacro == true) {
    std::cout << "Cancelled macro!" << std::endl;
  }

  // FishMech.stop();

  inLiftMacro = false;
}

bool detectedRing = false;
bool hardStopDetect = false;
double fishMechHardStop = 190;
float fishSpeed = 12;

void detectFishLimit() {
  while (1) {
    if (fishMechSensor.position(degrees) > fishMechHardStop && inLiftMacro == false) {
      FishMech.stop();
    }
    hardStopDetect = fishMechSensor.position(degrees) > fishMechHardStop ? true : false;
    fishSpeed = fishMechSensor.position(degrees) > fishMechHardStop ? 0 : 12;
  }
}

void testFunction() {
  std::cout << "chode alert chode alert" << std::endl;
  fishMechLoop(50);
}

void objectLost() {
  detectedRing = false;
}

void detectedObject() {
  if (intakeDetectMode != true) return;
  Controller.rumble("-");
  std::cout << "detected" << std::endl;
  //wait(300, msec);
  detectedRing = true;
  Intake.stop();
}

void intakeHardStop() {
  while (1) {
   // std::cout << "nearRish" << RingFilter.isNearObject() << std::endl;
   if (intakeDetectMode == true) {
    if (detectedRing == true) {
      std::cout << "detected" << std::endl;
     // detectedRing = true;
    if (!Controller.ButtonL2.pressing()) {
      Intake.stop();
    }
    } else if (RingFilter.isNearObject() == false) {
      detectedRing = false;
    }
    } else {
      detectedRing = false;
    }
     wait(1, msec);
  }
}

void detectMode() {
  if (detectModeDB == true) return;
  detectModeDB = true;

  intakeDetectMode = !intakeDetectMode;
  if (intakeDetectMode == true) {
    Controller.rumble("...");
  } else {
    Controller.rumble("-");
  }

  wait(50, msec);
  detectModeDB = false;
}

void raiseLift() {

  // pneumatics LiftClamp(Brain.ThreeWirePort.C);

  // if (inLiftMacro == true) return;
  // cancelMacro = false;
  // inLiftMacro = true;

  // Lift.spin(fwd, 12, volt);
  // waitUntil(liftDesiredPosition - Lift.position(degrees) < 250);
  // LiftClamp.open();
  // waitUntil(liftDesiredPosition - Lift.position(degrees) < 3 || cancelMacro == true);

  // if (cancelMacro == true) {
  //   std::cout << "Cancelled macro!" << std::endl;
  // }

  // Lift.stop();

  // inLiftMacro = false;
}

void toggleLiftClampF() {
  raiseLift();
}

void toggleLiftRatchetF() {
  pneumatics LiftRatchet(Brain.ThreeWirePort.D);

  if (liftRatchetDB == true) {return;}
  liftRatchetDB = true;

  toggleLiftRatchet = !toggleLiftRatchet;

  if (toggleLiftRatchet == true) {
    LiftRatchet.open();
  } else {
    LiftRatchet.close();
  }

  wait(50, msec);
  liftRatchetDB = false;
}

void toggleExtenderF() {
  pneumatics Extender(Brain.ThreeWirePort.E);

  if (extenderDB == true) {return;}
  extenderDB = true;

  toggleExtender = !toggleExtender;

  if (toggleExtender == true) {
    Extender.open();
  } else {
    Extender.close();
  }

  wait(50, msec);
  extenderDB = false;
}


// ALWAYS SET TO -1 DURING A COMPETITION MATCH
int autonSelected = 7;
std::string autonLabel = "Auton Selected: NONE";

class autonButton {
  private:
  int xPos, yPos, width, height, autonID;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char* label;

  public:

  autonButton(int x, int y, int width, int height, int autonId, bool state, vex::color offColor, vex::color onColor, const char* label) :
  xPos(x), yPos(y), width(width), height(height), autonID(autonId), state(state), offColor(offColor), onColor(onColor), label(label) {}

  void render() {
    vex::color renderColor = state ? onColor : offColor;
    Brain.Screen.drawRectangle(xPos, yPos, width, height, renderColor);
    Brain.Screen.printAt(xPos, yPos, label);
  }

  void detectClick() {
    if(Brain.Screen.pressing() && Brain.Screen.xPosition() >= xPos && Brain.Screen.xPosition() <= xPos + width &&
    Brain.Screen.yPosition() >= yPos && Brain.Screen.yPosition() <= yPos + width) { // clicked
      state = true;
      autonSelected = autonID;
      autonLabel = "Auton Selected: " + std::string(label);
    } else {
      state = false;
    }

  }
};

autonButton buttons[] = {
  autonButton(30, 30, 60, 60, 0, false, 0xE00000, 0x1f1c1c, "R_SAWP"),
  autonButton(150, 30, 60, 60, 1, false, 0x0000E0, 0x1f1c1c, "B_SAWP"),
  autonButton(270, 30, 60, 60, 2, false, 0x32a852, 0x1f1c1c, "R_OPP"),
  autonButton(390, 30, 60, 60, 3, false, 0xcf6c21, 0x1f1c1c, "B_OPP"),
  autonButton(30, 150, 60, 60, 4, false, 0xd1c821, 0x1f1c1c, "4-"),
  autonButton(150, 150, 60, 60, 5, false, 0x351ee3, 0x1f1c1c, "5-"),
  autonButton(270, 150, 60, 60, 6, false, 0xa11ee3, 0x1f1c1c, "testGS"),
  autonButton(390, 150, 60, 60, 7, false, 0xe3143e, 0x1f1c1c, "Skills")
};

void autonSelector() {
  while (1) {

    Brain.Screen.clearScreen();

    if (!Competition.isEnabled()) {
      for (int i = 0; i < 8; i++) {
        buttons[i].render();
        buttons[i].detectClick();
      }

      Brain.Screen.printAt(90, 125, autonLabel.c_str());
    }

    Brain.Screen.render();

    wait(20, msec);
  }
}

void pre_auton(void) {

  FrontLeft.setBrake(coast);
  TopLeft.setBrake(coast);
  BackLeft.setBrake(coast);
  FrontRight.setBrake(coast);
  TopLeft.setBrake(coast);
  BackRight.setBrake(coast);

  Intake.setBrake(coast);
  FishMech.setBrake(hold);
  
  LeftSide.resetPosition();
  RightSide.resetPosition();
  FishMech.resetPosition();

  Intake.setVelocity(100, pct);

  launch_task([&] {
    autonSelector();
  });

  //RingFilter.objectDetectThreshold(255);
  RingFilter.setLight(ledState::on);
  RingFilter.setLightPower(100);

  InertialSensor.calibrate();
  waitUntil(InertialSensor.isCalibrating() == false);

  Robot.initOdom();

 // Robot.setRobotCoordinates({-59, -54, 180});
}

void activateMotionChaining(bool reversed = false, float minSpeed = 5) {
  L_Settings.exitError = 8;
  LMogo_Settings.exitError = 8;
  A0_Settings.exitError = 20;
  A60_Settings.exitError = 20;
  A120_Settings.exitError = 20
  ;
  Linear.setExitError(8);
  Angular.setExitError(20);
  Robot.defaultMinSpeed = minSpeed;
  Robot.reversedChaining = reversed;
}

void deactivateMotionChaining(bool reversed = false) {
  L_Settings.exitError = 1;
  LMogo_Settings.exitError = 1;
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

void printCoordinates(bool reversed = false) {
    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false, reversed);
    printf("ATM right now: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
}

void IntakeToLift() {
  Intake.spin(reverse, 6.5, volt);

  waitUntil(RingFilter.isNearObject() == true);

  std::cout << "Found object" << std::endl;

  Intake.spin(fwd, 9, volt);

  wait(0.5, sec);
}

void macroDisrupter() {
  cancelMacro = true;
}

void liftClampChecker() {

  // pneumatics LiftClamp(Brain.ThreeWirePort.C);

  // while (1) {

  //   if (Lift.position(degrees) < 0) {
  //     LiftClamp.close();
  //   } else if (Lift.position(degrees) < 50) {
      
  //   }

  //   wait(10, msec);
  // }
}

void setBrakeType(brakeType brake) {
  FrontLeft.setBrake(brake);
  TopLeft.setBrake(brake);
  BackLeft.setBrake(brake);
  FrontRight.setBrake(brake);
  TopLeft.setBrake(brake);
  BackRight.setBrake(brake);

}

void skills() {

  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);


  // Fish Mech Macro
  launch_task([&] {
    intakeHardStop();
  });

  //-- Alliance Stake Score --//

  Intake.spin(reverse, 12, volt);

  wait(0.45, seconds);

  Intake.stop();

  FishMech.spin(forward, 3, volt);

  // launch_task([&] {
  //   fishMechLoop(150);
  // });

  //-- Mogo #1 Clamp --//

  Robot.DriveToPoint(0, 12.5, L_Settings, A0_Settings);

  Robot.TurnFor(90, A60_Settings, 800); // face the goal

  Linear.setMaxVoltages(10);

  printCoordinates();

  Robot.DriveFor(-25.5, L_Settings, true, 1000);

  Linear.setMaxVoltages(12);

  Mogo.open(); // grab first goal

  printCoordinates();

  //-- First 6 Ring Score  --//

  Robot.TurnFor(0, A60_Settings, 500); // face the ring

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(-20, 37, L_Settings, A0_Settings); // 1st ring

  Robot.TurnFor(-15, A0_Settings, 350);

  Robot.DriveToPoint(-58, 59, L_Settings, A0_Settings); // 2nd ring

  wait(900, msec);

  Robot.TurnFor(150, A60_Settings, 500);

  Angular.setMaxVoltages(12);
  Linear.setMaxVoltages(10);

  Robot.DriveToPoint(-47, 25, L_Settings, A0_Settings); // 3rd ring

  wait(1000, msec);

  Linear.setMaxVoltages(7.5);

  Robot.DriveToPoint(-47, 13, L_Settings, A0_Settings); // drive slow for 4th/5th rings

  printCoordinates();

  wait(1000, msec);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(14, L_Settings, false, 800);

 // Linear.setMaxVoltages(12);

  wait(750, msec);

  Robot.TurnFor(305, A60_Settings, 500);

  Robot.DriveToPoint(-65, 16, L_Settings, A0_Settings); // get 6th ring

  wait(750, msec);

  Robot.TurnFor(15, A120_Settings, 800);

 // wait(100, sec);

 Linear.setMaxVoltages(8);

  Robot.DriveToPoint(-68, -4.56, L_Settings, A0_Settings, 800, true); // back mogo into corner
  
  Mogo.close();

  wait(500, msec);

  printCoordinates();

  //-- Mogo #2 Clamp --//

  Linear.setMaxVoltages(12);

  Intake.spin(forward, 12, volt);

  Robot.DriveToPoint(-50.814, 17, L_Settings, A0_Settings);

  Linear.setMaxVoltages(10);

  Robot.DriveToPoint(0, 17, L_Settings, A0_Settings);

  Robot.TurnFor(-90, A60_Settings, 1000);

  Intake.stop();

  printCoordinates();

  Robot.DriveFor(-22, L_Settings, true, 1500);

  Mogo.open(); // Second Goal

  Linear.setMaxVoltages(12);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Robot.TurnFor(0, A60_Settings, 800);

  Robot.resetOdom();

  printCoordinates();

  Robot.DriveToPoint(0, 24, L_Settings, A0_Settings);

  wait(350, msec);

  Robot.TurnFor(55, A0_Settings, 800);

  Robot.DriveFor(10, L_Settings, false, 850);

  Robot.DriveToPoint(20.5, 75, L_Settings, A0_Settings);

  wait(1000, msec);

  Robot.TurnFor(180, A120_Settings, 800);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Robot.DriveToPoint(21, 12.5, L_Settings, A0_Settings);

  wait(800, msec);

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(19.5, -5, L_Settings, A0_Settings);

  wait(800, msec);

  Robot.DriveToPoint(19.5, -20, L_Settings, A0_Settings);

  wait(1000, msec);

  Robot.TurnFor(60, A0_Settings, 1000);

  Robot.DriveFor(15, L_Settings, false, 1000);

  wait(500, msec);

  Robot.DriveFor(-10, L_Settings, false, 1000);

  Robot.TurnFor(-50, A0_Settings, 800);

  Robot.DriveFor(-15, L_Settings, true, 1500);

  Mogo.close();

  Intake.spin(forward, 12, volt);

  printCoordinates();

  Robot.DriveFor(17.5, L_Settings, false);

  Robot.TurnFor(0, A0_Settings, 1000);

  Robot.DriveFor(46.5, L_Settings);

  Intake.spin(reverse, 11, volt);

  Robot.TurnFor(88, A60_Settings, 1000);

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
 });

  Robot.DriveFor(17.5, L_Settings, false, 1500);

  wait(1000, msec);

  FishMech.spin(reverse, 12, volt);
  Intake.spin(reverse, 4.5, volt);

  wait(750, msec);

  Robot.DriveFor(-15, L_Settings, false, 1000);



  // launch_task([&] {
  //   waitUntil(RingFilter.isNearObject() == true);
  //   Intake.stop();
 //});

//   Robot.DriveFor(20, L_Settings);

//   wait(1000, msec);

//  // Robot.DriveToPoint(29, 42, L_Settings, A0_Settings);

//   Robot.TurnFor(118, A60_Settings, 1000);

// // Robot.DriveFor(8, L_Settings, false, 800);

//   FishMech.spin(reverse, 12, volt);
//   Intake.spin(reverse, 4, volt);

//   wait(1500, msec);

//   fishMechLoop();

//   printCoordinates();

//   wait(100, sec);

  // Robot.DriveToPoint(20, 40, L_Settings, A0_Settings); // 1st ring

  // wait(1000, msec);

  // Robot.TurnFor(80, A60_Settings, 500);

  // Robot.DriveToPoint(50, 28, L_Settings, A0_Settings); // 2nd ring

  // wait(750, msec);

  // Robot.TurnFor(180, A60_Settings, 800);

  // Robot.DriveToPoint(48, 1, L_Settings, A0_Settings);

  // wait(750, msec);

  // Robot.DriveToPoint(48, -9, L_Settings, A0_Settings, 800);

  // wait(750, msec);

  // Robot.TurnFor(55, A0_Settings, 800);

  // Robot.DriveFor(9, L_Settings, false, 800);

  // printCoordinates();

  // wait(600, msec);

  // Robot.TurnFor(-20, A0_Settings, 800);

  // Robot.DriveFor(-14, L_Settings, true, 1000);

  // Intake.spin(forward, 12, volt);

  // Mogo.close();

  // wait(500, msec);

  // launch_task([&] {
  //   resetFishMech();
  //   Intake.spin(reverse, 10, volt);
  // });

  // Linear.setMaxVoltages(10);

  // Robot.DriveToPoint(52, 20, L_Settings, A0_Settings);

  // Robot.DriveToPoint(59, 54, L_Settings, A0_Settings);

  // waitUntil(RingFilter.isNearObject() == true);

  // Intake.stop();

  // Robot.TurnFor(100, A60_Settings, 1000);

  // Robot.DriveFor(3, L_Settings, false, 700);

  // FishMech.spin(reverse, 12, volt);

  // setBrakeType(hold);

  // wait(1000, msec);

  // resetFishMech();
  // setBrakeType(coast);
  // Intake.spin(reverse, 12, volt);
}

void testGainScheduling() {
  pneumatics Mogo(Brain.ThreeWirePort.G);

  Robot.DriveFor(72, L_Settings, true);
  Controller.rumble("-");
}

void tuneOffsets() {
  Robot.TurnFor(180, A120_Settings, 1000);
  
  Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
  printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
}

// DataSink_Motor<vex::motor*, DATA_TYPES> kalman("KalmanDataMotor", 
// {
// Bucees::createDataCapture(&TopLeft, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&TopRight, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&FrontLeft, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&FrontRight, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&BackLeft, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&BackRight, MOTOR_VELOCITY_RPM),}, 
// 50);

// DataSink_Robot<Bucees::Robot*, DATA_TYPES> kalman2("KalmanDataOdom",
// {
//  Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_X), 
//  Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_Y), 
//  Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_ROTATION_RAD), 
// }, 50);

// void testKalmanFilter() {

//   //Linear.setMaxVoltages(5);
// activateMotionChaining(false, 9);
//   // Robot.DriveToPoint(0, 24, L_Settings, A0_Settings, 0, true);
//  // Controller.rumble("-");
//   Robot.DriveToPoint(-24, 40, L_Settings, A60_Settings, 0, true);
//   deactivateMotionChaining();
// //  Controller.rumble("-");
//   Robot.DriveToPoint(-48, 40, L_Settings, A0_Settings, 0, true);
//  // Controller.rumble("-");
// }

// void testMotions() {
//   //L_Settings.maxVoltages = 9;
   
//   activateMotionChaining(false, 9);
//   Robot.DriveToPoint(0, 24, L_Settings, A0_Settings);
//  // Controller.rumble("-");
//   Robot.DriveToPoint(-24, 40, L_Settings, A60_Settings);
//   deactivateMotionChaining();
// //  Controller.rumble("-");
//   Robot.DriveToPoint(-48, 40, L_Settings, A0_Settings);
//  // Controller.rumble("-");
//   return;
// }

void newBlueOpp() {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  FishMech.spin(forward, 3, volt);

  Robot.DriveToPoint(0, 37.5, LOdom_Settings, A0_Settings);

  printCoordinates();

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(5, 45.5, LOdom_Settings, A0_Settings, 1000);

  printCoordinates();

  Robot.TurnFor(47.5, A0_Settings, 700);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(-30, LOdom_Settings, true, 1250);

  Mogo.open();

 // Intake.spin(reverse, 12, volt);

  wait(200, msec);

  Robot.TurnFor(90, A60_Settings, 500);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(12, 29, LOdom_Settings, A0_Settings);

  wait(500, msec);

  Robot.TurnFor(10.5, A0_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(14.5, L_Settings, false, 1000);

  wait(0.2, seconds);

  Linear.setMaxVoltages(6);

  Robot.DriveFor(-17, L_Settings, true, 1000);

  Robot.TurnFor(-183, A120_Settings, 1000);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(26, L_Settings, false, 1500);

  printCoordinates();

  wait(1.15, seconds);

  Robot.TurnFor(-90, A120_Settings, 1000);

  Robot.DriveFor(45, L_Settings, false);

  wait(100, seconds); // 0.35
}

void newRedOpp() {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
  });

  FishMech.spin(forward, 3, volt);

  //Extender.open();

  Robot.DriveToPoint(0, 37.5, LOdom_Settings, A0_Settings);

  printCoordinates();

  //Linear.setMaxVoltages(10);

  Intake.spin(reverse, 9.5, volt);

  Robot.DriveToPoint(-4, 46.5, LOdom_Settings, A0_Settings, 1000);

  printCoordinates();

//   Extender.close();

  Robot.TurnFor(-48, A0_Settings, 700);

  Robot.DriveFor(-28.5, L_Settings, true, 1200);

  Mogo.open();

  wait(200, msec);

  Robot.TurnFor(-90, A60_Settings, 500);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(-18, 29, LOdom_Settings, A0_Settings);

  wait(500, msec);

  Robot.TurnFor(-10.5, A0_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(17, L_Settings, false, 1000);

  wait(0.2, seconds);

  Linear.setMaxVoltages(5);

  Robot.DriveFor(-16, L_Settings, true, 1000);

  Robot.TurnFor(178, A120_Settings, 1000);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(25, L_Settings, false, 1500);

  printCoordinates();

  wait(0.75, seconds);

  Robot.TurnFor(90, A120_Settings, 1000);

  Intake.stop();

  Robot.DriveFor(45, L_Settings, false);

  wait(100, seconds); // 0.35
}

void newBAWP() {

 // Robot.setRobotCoordinates({-59, -54, 180});

  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  Robot.DriveToPoint(0, -35, LOdom_Settings, A0_Settings, 0, true); //htbht
 // deactivateMotionChaining(true);
  Linear.setMaxVoltages(6.25);
  Robot.DriveToPoint(-7.65, -51.75, L_Settings, A0_Settings, 0, true); // htbht
  printCoordinates();
 // Robot.DriveFor(-1.5, LOdom_Settings, true, 500); //htbht
  Linear.setMaxVoltages(8.5);

  //wait(100, msec);

  Mogo.open();

  wait(500, msec);

  Intake.spin(reverse, 12, volt);
  printCoordinates();
  launch_task([&] {
    wait(300, msec); // htbht
    fishMechLoop(200);
  });

  Robot.DriveToPoint(-9, -25, LOdom_Settings, A0_Settings); // htbht
  Linear.setMaxVoltages(12);

  wait(0.2, sec);

  Robot.DriveFor(-4, LOdom_Settings, false, 500);

  wait(0.5, sec);

  Intake.stop();

  Mogo.close();

  // Robot.TurnFor(-90, A60_Settings, 1000);

  // wait(300, msec);

  Robot.TurnFor(90, A60_Settings, 1350);

  // make sure to not get stuck by the stjupid ring

  Robot.DriveFor(-25, L_Settings, false, 2000);

  Mogo.open(); // Grab second goal

  printCoordinates();

  Robot.TurnFor(0, A60_Settings, 800);

 //Robot.resetOdom();

  Robot.DriveFor(18, L_Settings);

  Robot.TurnFor(-90, A0_Settings, 1000, false);

  IntakeLift.open();

  printCoordinates();

  Robot.DriveFor(15, L_Settings); // has to be hella tuned

  Intake.spin(reverse, 12, volt);

  IntakeLift.close();

  wait(0.45, seconds);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-5, LOdom_Settings, true, 500);

  Linear.setMaxVoltages(7);

  wait(300, msec);

  Robot.DriveFor(-14.5, LOdom_Settings, true, 750);

  Intake.spin(reverse, 12, volt);

  Robot.TurnFor(-150, A120_Settings, 1000);

  Linear.setMaxVoltages(7.5);

  FishMech.spin(reverse, 3, volt);

  Robot.DriveFor(35, L_Settings, false, 10000);

  wait(100, sec);
}

void testRAWP() {

 // Robot.setRobotCoordinates({-59, -54, 180});

  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  Robot.DriveToPoint(0, -30, LOdom_Settings, A0_Settings, 0, true); //htbht
 // deactivateMotionChaining(true);
  Linear.setMaxVoltages(6.25);
  Robot.DriveToPoint(7.65, -49.5, L_Settings, A0_Settings, 0, true); // htbht
  printCoordinates();
 // Robot.DriveFor(-1.5, LOdom_Settings, true, 500); //htbht
  Linear.setMaxVoltages(8.5);

  //wait(100, sec);

  Mogo.open();

  wait(400, msec);

  Intake.spin(reverse, 12, volt);
  printCoordinates();
  launch_task([&] {
    wait(300, msec); // htbht
    fishMechLoop(200);
  });

  Robot.DriveToPoint(9, -25, LOdom_Settings, A0_Settings); // htbht
  Linear.setMaxVoltages(12);

  wait(0.2, sec);

  Robot.DriveFor(-5, LOdom_Settings, false, 500);

  wait(0.55, sec);

  Intake.stop();

  Mogo.close();

  Robot.TurnFor(-90, A60_Settings, 800);

  Robot.DriveFor(-34, L_Settings, true, 1500);

  //Robot.DriveToPoint(34, -28.5, LOdom_Settings, A0_Settings, 0, true); // htbht

  Mogo.open(); // Grab second goal

  printCoordinates();

  Robot.TurnFor(0, A60_Settings, 800);

  wait(0.2, seconds);

  Robot.DriveFor(24, L_Settings, true);

  Robot.TurnFor(90, A0_Settings, 1000, false);

  IntakeLift.open();

  Robot.DriveFor(9.5, L_Settings, true, 700); // has to be hella tuned

  Intake.spin(reverse, 12, volt);

  IntakeLift.close();

  wait(0.45, seconds);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-5, LOdom_Settings, true, 500);

  Linear.setMaxVoltages(7);

  wait(300, msec);

  Robot.DriveFor(-14.5, LOdom_Settings, true, 750);

  Intake.spin(reverse, 12, volt);
  FishMech.spin(reverse, 3, volt);

  Robot.TurnFor(150, A120_Settings, 1000);

  Linear.setMaxVoltages(7);

  Robot.resetOdom();

  wait(0.1, sec);

  Robot.DriveFor(46.5, L_Settings, true, 20000);

  wait(100, sec);
}

DataSink_Robot<Bucees::Robot*, DATA_TYPES> offsetCalculator("offsetVerify",
{
 Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_X), 
 Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_Y), 
}, 50);

bool calculateStop = false;

void calculateOffSets() {
  // offsetCalculator.start();
  std::cout << std::fixed << "\033[1mCopy this:\033[0m\n\\left[";
  LeftSide.spin(reverse, 9, volt);
  RightSide.spin(forward, 9, volt);
  launch_task([&] {
    while (1) {
      Bucees::Coordinates current = Robot.getRobotCoordinates();
      std::cout << "\\left(" << current.x << "," << current.y << "\\right),";

      if (calculateStop == true) break;
     
      wait(20, msec);
    }
  });
  wait(4, seconds);
  calculateStop = true;
  LeftSide.stop(hold);
  RightSide.stop(hold);
  std::cout << "\b\\right]" << std::endl;
  // offsetCalculator.stop();
  // offsetCalculator.parseData();
}

void autonomous(void) {

  switch (autonSelected) {

    case -1:
    return;

    case 0:
    testRAWP();
    return;

    case 1:
    newBAWP();
    return;

    case 2:
    newRedOpp();
    return;

    case 3:
    newBlueOpp();
    return;

    case 4:
    calculateOffSets();
   // testAWP();
    return;

    case 5:
    //newRedOpp();
    return;

    case 6:
    //testKalmanFilter();
    return;

    case 7:
   //tuneOffsets();
    skills();
    return;
  }
}

double driveCurve(double x, double scale) {
  if (scale != 0) {
    return (scale * pow(1.01, x) - scale);
    //return (pow(2.718, (scale * (std::fabs(x) - 12))) / 1000) * x;
  }
  return x;
}

double intakeSpeed = 12;
double reverseIntakeSpeed = 12;

void usercontrol(void) {

  Robot.setRobotCoordinates({0.17, 24.05, 44.88});

  // optical:
  RingFilter.objectDetected(detectedObject);
  RingFilter.objectLost(objectLost);

  // pneumatics:
  Controller.ButtonB.pressed(toggleMogoF);
  Controller.ButtonY.pressed(toggleExtenderF);
 // Controller.ButtonLeft.pressed(detectMode);
  Controller.ButtonDown.pressed(detectMode);
  Controller.ButtonUp.pressed(toggleLiftRatchetF);
  Controller.ButtonA.pressed(toggleIntakeLiftF); // intake lift
  Controller.ButtonL1.pressed(resetFishMech);
  Controller.ButtonR1.pressed(macroDisrupter);

  // launch_task([&] {
  //   liftClampChecker();
  // });

  launch_task([&] {
    detectFishLimit();
  });

  launch_task([&] {
    intakeHardStop();
  });

  //Robot.setRobotCoordinates({8, 7, -92});
  
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

    LeftSide.spin(fwd, LeftJoystickPosition + RightJoystickPosition, volt);
    RightSide.spin(fwd, LeftJoystickPosition - RightJoystickPosition, volt);

    intakeSpeed = intakeDetectMode ? 11 : 12;
    reverseIntakeSpeed = intakeDetectMode ? 6 : 12;

    if (Controller.ButtonL2.pressing()) {
      Intake.spin(forward, reverseIntakeSpeed, volt);
    } else if (Controller.ButtonR2.pressing() && detectedRing == false) {
      Intake.spin(reverse, intakeSpeed, volt);
    } else {
      Intake.stop();
    }

    if (Controller.ButtonR1.pressing()) {
      Intake.spin(reverse, 5.5, volt);
      FishMech.spin(reverse, fishSpeed, volt);
    } else if (inLiftMacro == false) {
      FishMech.stop();
    }

    Brain.Screen.printAt(50, 25, "BackLeft Temp: %f", BackLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 50, "BackRight Temp: %f", BackRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 75, "TopLeft Temp: %f", TopLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 100, "TopRight Temp: %f", TopRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 125, "FrontLeft Temp: %f", FrontLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 150, "FrontRight Temp: %f", FrontRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 175, "Intake Temp: %f", Intake.temperature(temperatureUnits::fahrenheit));

    Brain.Screen.drawImageFromFile("Brain_Screen_Logo.png", 0, 0);

    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
   // std::cout << "Color: " << RingFilter.isNearObject() << std::endl;
    printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);

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