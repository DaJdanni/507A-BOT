#include "vex.h"
using namespace vex;

template <class F>
vex::task launch_task(F&& function) {
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

int waitTimeFilter = 170;
bool blueFilter;
bool redFilter;

void filter() {
  if (RingFilter.hue() > 150 && redFilter == true) {return;}
  if (RingFilter.hue() < 15 && blueFilter == true) {return;}
//  if (RingFilter.hue() < 10) {
    wait(waitTimeFilter, msec);
    Intake.spin(forward, 12, volt);
    wait(300, msec);
    Intake.spin(reverse, 12, volt);
 //  }
}


// void skills() {
//   pneumatics Clamp(Brain.ThreeWirePort.A);
//   pneumatics Doinker(Brain.ThreeWirePort.B);
//   pneumatics goalRush(Brain.ThreeWirePort.C);

//   //-- Alliance Stake Score --//

//   Intake.spin(reverse, 12, volt);

//   wait(0.45, seconds);

//   Intake.stop();

//   FishMech.spin(forward, 1, volt);

//   // launch_task([&] {
//   //   fishMechLoop(150);
//   // });

//   //-- Clamp #1 Clamp --//

//   Robot.DriveToPoint(0, 11, L_Settings, A0_Settings);

//   Robot.TurnFor(90, A60_Settings, 800); // face the goal

//   Linear.setMaxVoltages(10);

//   printCoordinates();

//   Robot.DriveFor(-25.5, L_Settings, true, 1000);

//   Linear.setMaxVoltages(12);

//   Clamp.open(); // grab first goal

//   printCoordinates();

//   //-- First 6 Ring Score  --//

//   Robot.TurnFor(0, A60_Settings, 500); // face the ring

//   Intake.spin(reverse, 12, volt);

//   Robot.DriveToPoint(-20, 36, L_Settings, A0_Settings); // 1st ring

//   Robot.TurnFor(-15, A0_Settings, 350);

//   Robot.DriveToPoint(-56, 59, L_Settings, A0_Settings); // 2nd ring

//   wait(900, msec);

//   Robot.TurnFor(150, A60_Settings, 500);

//   Angular.setMaxVoltages(12);
//   Linear.setMaxVoltages(9);

//   Intake.spin(reverse, 12, volt);

//   Robot.DriveToPoint(-46.5, 24.5, L_Settings, A0_Settings); // 3rd ring

//   wait(600, msec);

//  // Linear.setMaxVoltages(6);

//   Robot.DriveToPoint(-47, 12.5, L_Settings, A0_Settings); // 4th ring

//   wait(800, msec);

//   Robot.DriveToPoint(-47, -3, L_Settings, A0_Settings); // 5th ring

//   wait(750, msec);

//   Robot.TurnFor(305, A60_Settings, 500);

//   Linear.setMaxVoltages(12);

//   Robot.DriveToPoint(-65, 17, L_Settings, A0_Settings); // get 6th ring

//   wait(750, msec);

//   Robot.TurnFor(15, A120_Settings, 800);

//  // wait(100, sec);

//  Linear.setMaxVoltages(8);

//   Robot.DriveToPoint(-68, -4.56, L_Settings, A0_Settings, 800, true); // back Clamp into corner
  
//   Clamp.close();

//   wait(500, msec);

//   printCoordinates();

//   //-- Clamp #2 Clamp --//

//   Linear.setMaxVoltages(12);

//   Intake.spin(forward, 12, volt);

//   Robot.DriveToPoint(-50.814, 19, L_Settings, A0_Settings);

//   Linear.setMaxVoltages(10);

//   Robot.DriveToPoint(0, 19, L_Settings, A0_Settings);

//   Robot.TurnFor(-90, A60_Settings, 1000);

//   Intake.stop();

//   printCoordinates();

//   Robot.DriveFor(-22, L_Settings, true, 1500);

//   Clamp.open(); // Second Goal

//   Linear.setMaxVoltages(12);

//   Intake.spin(reverse, 12, volt);

//   printCoordinates();

//   Robot.TurnFor(0, A60_Settings, 800);

//   Robot.resetOdom();

//   printCoordinates();

//   Robot.DriveToPoint(0, 26, L_Settings, A0_Settings);

//   wait(350, msec);

//   Robot.TurnFor(55, A0_Settings, 800);

//   Robot.DriveFor(9, L_Settings, false, 850);

//   Robot.DriveToPoint(20.5, 75, L_Settings, A0_Settings);

//   wait(1000, msec);

//   Robot.TurnFor(180, A120_Settings, 800);

//   Intake.spin(reverse, 12, volt);

//   printCoordinates();

//   Linear.setMaxVoltages(8.5);

//   Robot.DriveToPoint(20, 12.5, L_Settings, A0_Settings); // 3rd

//   wait(600, msec);

//  // Linear.setMaxVoltages(2.5);

//   Robot.DriveToPoint(20, -5, L_Settings, A0_Settings); // 4th

//   wait(800, msec);

//   Robot.DriveToPoint(20, -24, L_Settings, A0_Settings, 2000); // 5th

//   wait(750, msec);

//   Linear.setMaxVoltages(12);

//   wait(1000, msec);

//   Robot.TurnFor(60, A0_Settings, 1000);

//   Robot.DriveFor(15, L_Settings, false, 1000); // 6th ring

//   wait(500, msec);

//   Robot.DriveFor(-10, L_Settings, false, 1000);

//   Robot.TurnFor(-50, A0_Settings, 800);

//   Robot.DriveFor(-15, L_Settings, true, 1500);

//   Clamp.close(); 

//   Intake.spin(forward, 12, volt); // Clamp #2

//   printCoordinates();

//   Robot.DriveFor(17.5, L_Settings, false, 1000);

//   Robot.TurnFor(0, A0_Settings, 1000);

//   Robot.DriveFor(52.5, L_Settings);

//   Intake.spin(reverse, 10.5, volt);

//   printCoordinates();

//   launch_task([&] {
//     waitUntil(RingFilter.isNearObject() == true);
//     Intake.stop();
//  });

//   Robot.DriveToPoint(-1, 65, L_Settings, A0_Settings); // 1st ring

//   printCoordinates();

//   wait(350, msec);

//   //Robot.DriveFor(5, L_Settings, true);

//   Robot.TurnFor(130, A120_Settings, 1000);

//   Linear.setMaxVoltages(8);

//   Robot.DriveFor(-43, L_Settings, true, 3000);

//   Clamp.open(); // Clamp #3

//   Robot.TurnFor(-140, A120_Settings, 800);

//   Intake.spin(reverse, 12, volt);

//   printCoordinates();

//   Linear.setMaxVoltages(12);

//   Robot.DriveFor(31, L_Settings); // 2nd ring

//   Robot.TurnFor(-90, A60_Settings, 1000);

//   Robot.DriveFor(20, L_Settings); // 3rd ring

//   Robot.TurnFor(0, A60_Settings, 1000);

//   Robot.DriveFor(25, L_Settings); // 4th ring

//   Linear.setMaxVoltages(7.5);

//   Robot.DriveFor(-16, L_Settings);

//   Robot.TurnFor(-90, A60_Settings, 1000);

//   wait(500, msec);

//   Linear.setMaxVoltages(12);

//   Robot.DriveFor(13, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 1000);

//   Robot.DriveFor(18, L_Settings); // 5th ring

//   wait(100, sec);

// }

void startDrivetrain(double speed) {
  LeftSide.spin(fwd, speed, volt);
  RightSide.spin(fwd, speed, volt);
}

void stopDrivetrain(brakeType brake) {
  LeftSide.stop(brake);
  RightSide.stop(brake);
}

void jerkIntake() {
  Intake.spin(fwd, 12, volt);
  wait(10, msec);
  Intake.spin(reverse, 12, volt);
  wait(10, msec);
  Intake.stop();
}

void limitLadyBrown() {
  while (1) {
    float rotationPosition = (ladyBrown1.position(degrees) + ladyBrown2.position(degrees)) / 2;
    if (rotationPosition > 525) {
      ladyBrown.stop();
    }
    wait(10, msec);
  }
}

void skills() {
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  ladyBrown1.resetPosition();
  ladyBrown2.resetPosition();

  launch_task([&] {
    limitLadyBrown();
  });

  activateMotionChaining(false, 5.5);

  Linear.setMaxVoltages(7.5); // too fast = harder timing

  Intake.spin(reverse, 12, volt);

  wait(400, msec);

  launch_task([&] {lBPid(65);}); // first ring

  Robot.DriveToPoint(0, 7.5, L_Settings, A0_Settings);

  Robot.DriveToPoint(24, 40, L_Settings, A0_Settings, 0, false);

  deactivateMotionChaining();

  Robot.DriveToPoint(40, 61, L_Settings, A0_Settings, 1000, false);

  Robot.TurnFor(90, A60_Settings, 800);

  Robot.DriveToPoint(57, 63.5, L_Settings, A0_Settings, 0, false);

  Robot.TurnFor(90, A60_Settings, 500);

  Intake.stop(coast);

  wait(100, msec);

  lBPid(140); // first ring

  launch_task([&] {
  Intake.spin(reverse, 12, volt);

  wait(500, msec);

  Intake.stop();
  });

  Linear.setMaxVoltages(5.5);
  
  Robot.DriveFor(9999, L_Settings, false, 3000, true);

  wait(500, msec);

  ladyBrown.spin(forward, 12, volt);

  wait(800, msec);

  ladyBrown.stop(hold);

  lBPid(68.5); // first ring

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  wait(700, msec);

  Intake.stop(coast);

  wait(100, msec);

  ladyBrown.spin(forward, 12, volt);
  Intake.stop();

  wait(800, msec);

  launch_task([&] {lBPid(0);}); // first ring

  Robot.waitChassis();

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-10, L_Settings, false, 800);

  Intake.spin(reverse, 12, volt);

  Robot.TurnFor(0, A60_Settings, 500);

  //-- GOAL 1 SECTION --//

  Robot.DriveToPoint(47.5, 100, L_Settings, A0_Settings); // ring 1

  Intake.stop();

  Linear.setMaxVoltages(12);

  activateMotionChaining(false, 9);

  Robot.DriveToPoint(35, 47, L_Settings, A0_Settings, 0, true); // goal pt1

  deactivateMotionChaining();

  Linear.setMaxVoltages(7.5);

  Robot.DriveToPoint(24, 17, L_Settings, A0_Settings, 0, true); // goal

  Clamp.open();

  wait(300, msec);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  wait(300, msec);

  Robot.DriveToPoint(47, 42, L_Settings, A0_Settings); // ring 3

  wait(450, msec);

  Robot.TurnFor(180, A120_Settings, 800);

  Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(45, 0, L_Settings, A0_Settings); // ring 4/5

  wait(750, msec);

  Robot.TurnFor(45, A120_Settings, 800);

  Robot.DriveToPoint(60, 18, L_Settings, A0_Settings); // ring 6

  Robot.DriveToPoint(65, -1, L_Settings, A60_Settings, 0, true); // put it there kid

  Robot.TurnFor(-45, A0_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-12.5, L_Settings, false, 1000);

  Intake.stop();

  Clamp.close();

  wait(200, msec);

  Robot.DriveToPoint(26.2, 22, L_Settings, A0_Settings);

  Robot.DriveToPoint(-23, 22, L_Settings, A0_Settings);
}

void disruptBlueRing(bool elims) {

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  redFilter = true;
  waitTimeFilter = 150;
 // RingFilter.objectDetected(filter);

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
    return;
  });

  //FishMech.spin(forward, 3, volt);

  activateMotionChaining(false, 7);

  Robot.DriveToPoint(0, 37.5, L_Settings, A0_Settings);

  Intake.spin(reverse, 10.5, volt);

  deactivateMotionChaining();

  Robot.DriveToPoint(4.5, 42.5, L_Settings, A0_Settings);

  printCoordinates();

  Robot.TurnFor(47.5, A0_Settings, 700);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(-24, LOdom_Settings, true, 1250);

  Clamp.open();

 // Intake.spin(reverse, 12, volt);

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  Robot.TurnFor(90, A60_Settings, 500);

  printCoordinates();

  Robot.DriveToPoint(13.5, 29, L_Settings, A0_Settings);

  wait(400, msec);
  Robot.TurnFor(0, A0_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Robot.DriveFor(18.5, L_Settings, false, 1000);

  wait(0.2, seconds);

  Robot.DriveFor(-17, L_Settings, true, 1000);

  Robot.TurnFor(-174, A120_Settings, 1000);

  Robot.DriveFor(25, L_Settings, false, 1500);

  printCoordinates();

  wait(0.75, seconds);

  if (elims != true) {
      Robot.TurnFor(-45, A120_Settings, 1000);

      Linear.setMaxVoltages(7);

      Robot.DriveFor(45, L_Settings, false);

      wait(100, seconds); // 0.35

  } else {

    Intake.stop();

    Robot.TurnFor(-87.5, A120_Settings, 800);

    Linear.setMaxVoltages(11);

    Robot.DriveFor(50, L_Settings, false); // get to other corner

    wait(500, sec);

    Robot.TurnFor(-90, A0_Settings, 200);

    wait(500, msec);

    Robot.DriveFor(40, L_Settings, false);

    wait(100, seconds); // 0.35
  }
}

void disruptRedRing(bool elims) {
  pneumatics Clamp(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  waitTimeFilter = 170;
  blueFilter = true;
  //RingFilter.objectDetected(filter);

       launch_task([&] {
        fishMechLoop(215);
      });

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
    return;
  });

  //Extender.open();

  Robot.DriveToPoint(0, 37.5, LOdom_Settings, A0_Settings);

  printCoordinates();

  //Linear.setMaxVoltages(10);

  Intake.spin(reverse, 10.5, volt);

  Robot.DriveToPoint(-4.15, 47, LOdom_Settings, A0_Settings, 1250);

  printCoordinates();

//   Extender.close();

  Robot.TurnFor(-48, A0_Settings, 700);

  Robot.DriveFor(-28.5, L_Settings, true, 1200);

  Clamp.open();

  wait(200, msec);

  Robot.TurnFor(-90, A60_Settings, 500);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-18, 29, LOdom_Settings, A0_Settings);

  wait(500, msec);

  Robot.TurnFor(-10.5, A0_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(17.5, L_Settings, false, 1000);

  wait(0.2, seconds);

  Linear.setMaxVoltages(5);

  Robot.DriveFor(-16, L_Settings, true, 1000);

  Robot.TurnFor(178, A120_Settings, 1000);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(25, L_Settings, false, 2000);

  printCoordinates();

  wait(0.75, seconds);

 // fishMechLoop(200);

  if (elims != true) {
    Robot.TurnFor(45, A120_Settings, 1000);

    //FishMech.stop();

    // launch_task([&] {
    //   fishMechLoop(215);
    // });

    Linear.setMaxVoltages(7);

    Robot.DriveFor(45, L_Settings, false);

    wait(100, seconds); // 0.35
  } else {

    Intake.stop();

    Robot.TurnFor(90, A120_Settings, 1000);

    Linear.setMaxVoltages(10);

    Robot.DriveFor(45, L_Settings, false); // get to other corner

    wait(500, sec);

    wait(1000, msec);

    Robot.DriveFor(30, L_Settings, false);

    wait(100, seconds); // 0.35
  }
}

void goalRushRed(bool elims) { // RETUNE

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  // launch_task([&] {
  //   lBPid(67);
  //   wait(200, msec);
  //   lBPid(190);
  // });

  launch_task([&] {
  waitUntil(GoalDetector.objectRawSize() == -1 && GoalDetector.objectDistance(inches) < 1.75);
  goalRush.open();
  std::cout << "clamp" << std::endl;

  });

  Intake.spin(reverse, 12, volt);

  Doinker.open();

  Linear.setMaxVoltages(11.5);
  Robot.DriveToPoint(0, 37, L_Settings, A60_Settings);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 13.5, L_Settings, A0_Settings, 1500, true, true);

  wait(100, msec);
  Intake.stop();
  wait(600, msec);;
  goalRush.close();
  wait(100, msec);
  Doinker.close();

  Robot.waitChassis();

  wait(500, msec);

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(7);

  Robot.DriveToPoint(-7.5, 36.5, L_Settings, A0_Settings, 2000, true);
  Linear.setMaxVoltages(12);

  Clamp.open();

  wait(200, msec);
  
  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(0, -5, L_Settings, A0_Settings);

  printCoordinates();

  Robot.TurnFor(Robot.getAbsoluteHeading() + 12.5, A0_Settings, 800);
  Robot.DriveFor(-16.5, L_Settings, false, 1000);
  
  wait(200, msec);

  Clamp.close();

  LeftSide.spin(fwd, 5.5, volt);
  RightSide.spin(fwd, 5.5, volt);

  wait(450, msec);

  LeftSide.spin(fwd, 12, volt);
  RightSide.spin(fwd, 12, volt);
  
  wait(850, msec);

  LeftSide.stop();
  RightSide.stop();

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-12.5, L_Settings, false, 0, true);

  Intake.stop();

  Robot.waitChassis();

  Robot.DriveToPoint(36.5, 7.5, L_Settings, A60_Settings, 0, true);

  Robot.TurnFor(180, A60_Settings, 800);

  Linear.setMaxVoltages(7.5);

  Robot.DriveToPoint(24.5, 34.5, L_Settings, A60_Settings, 0, true);

  Clamp.open();


  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  wait(100, sec);
}

void goalRushBlue(bool elims) {

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  // launch_task([&] {
  //   lBPid(67);
  //   wait(200, msec);
  //   lBPid(190);
  // });

  launch_task([&] {
  waitUntil(GoalDetector.objectRawSize() == -1 && GoalDetector.objectDistance(inches) < 1.75);
  goalRush.open();
  std::cout << "clamp" << std::endl;

  });

  Intake.spin(reverse, 12, volt);

  Doinker.open();

  Linear.setMaxVoltages(11.5);
  Robot.DriveToPoint(0, 37, L_Settings, A60_Settings);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 13.5, L_Settings, A0_Settings, 1500, true, true);

  wait(100, msec);
  Intake.stop();
  wait(600, msec);;
  goalRush.close();
  wait(100, msec);
  Doinker.close();

  Robot.waitChassis();

  wait(500, msec);

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(7);

  Robot.DriveToPoint(-7.5, 36.5, L_Settings, A0_Settings, 2000, true);
  Linear.setMaxVoltages(12);

  Clamp.open();

  wait(200, msec);
  
  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(0, -5, L_Settings, A0_Settings);

  printCoordinates();

  Robot.TurnFor(Robot.getAbsoluteHeading() + 12.5, A0_Settings, 800);
  Robot.DriveFor(-16.5, L_Settings, false, 1000);
  
  wait(200, msec);

  Clamp.close();

  LeftSide.spin(fwd, 5.5, volt);
  RightSide.spin(fwd, 5.5, volt);

  wait(450, msec);

  LeftSide.spin(fwd, 12, volt);
  RightSide.spin(fwd, 12, volt);
  
  wait(850, msec);

  LeftSide.stop();
  RightSide.stop();

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-12.5, L_Settings, false, 0, true);

  Intake.stop();

  Robot.waitChassis();

  Robot.DriveToPoint(36.5, 7.5, L_Settings, A60_Settings, 0, true);

  Robot.TurnFor(180, A60_Settings, 800);

  Linear.setMaxVoltages(7.5);

  Robot.DriveToPoint(24.5, 34.5, L_Settings, A60_Settings, 0, true);

  Clamp.open();


  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  wait(100, sec);
}

// void safeSoloRedAWP(bool elims) {
//   pneumatics Clamp(Brain.ThreeWirePort.G);
//   pneumatics IntakeLift(Brain.ThreeWirePort.F);
//   pneumatics Extender(Brain.ThreeWirePort.E);

//   FishMech.spin(forward, 3, volt);

//   Robot.DriveFor(-28.5, L_Settings, true);

//   Clamp.open();

//   wait(150, msec);

//   Intake.spin(reverse, 12, volt);

//   wait(500, msec);

//   Robot.TurnFor(-90, A60_Settings, 800);

//   Robot.DriveFor(21.5, L_Settings, 1000);

//   Linear.setMaxVoltages(7.5);

//   Robot.DriveFor(-16, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Linear.setMaxVoltages(12);

//   Robot.DriveFor(19.5, L_Settings);

//   Robot.TurnFor(90, A60_Settings, 800);

//   Clamp.close();

//   launch_task([&] {
//     waitUntil(RingFilter.isNearObject() == true && RingFilter.hue() < 15);
//     Intake.stop();
//   });

//   Linear.setMaxVoltages(8.5);

//   Robot.DriveFor(27, L_Settings);

//   Linear.setMaxVoltages(12);

//   Intake.spin(reverse, 10.5, volt);

//   Robot.DriveFor(24, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Robot.DriveFor(-21, L_Settings);

//   Clamp.open();

//  // wait(150, msec);

//   Intake.spin(reverse, 11, volt);

// //  wait(150, msec);

//   Robot.TurnFor(90, A60_Settings, 800);

//   Robot.DriveFor(21, L_Settings);

//   Linear.setMaxVoltages(7.5);

//   Robot.DriveFor(-27.5, L_Settings);

//   launch_task([&] {
//     fishMechLoop(200);
//   });

//   Robot.TurnFor(-135, A120_Settings, 800);

//   FishMech.spin(reverse, 3, volt);

//   Robot.DriveFor(10, L_Settings);  
// }

// void safeSoloBlueAWP(bool elims) {
//   pneumatics Clamp(Brain.ThreeWirePort.G);
//   pneumatics IntakeLift(Brain.ThreeWirePort.F);
//   pneumatics Extender(Brain.ThreeWirePort.E);

//   Robot.DriveFor(-28.5, L_Settings, true);

//   Clamp.open();

//   wait(150, msec);

//   Intake.spin(reverse, 12, volt);

//   wait(500, msec);

//   Robot.TurnFor(90, A60_Settings, 800);

//   Robot.DriveFor(21.5, L_Settings);

//   Linear.setMaxVoltages(6);

//   Robot.DriveFor(-16, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Linear.setMaxVoltages(12);

//   Robot.DriveFor(19.5, L_Settings);

//   Robot.TurnFor(-90, A60_Settings, 800);

//   Clamp.close();

//   launch_task([&] {
//     waitUntil(RingFilter.isNearObject() == true && RingFilter.hue() > 100);
//     Intake.stop();
//   });

//   Linear.setMaxVoltages(8.5);

//   Robot.DriveFor(24, L_Settings);

//   Linear.setMaxVoltages(12);

//   Intake.spin(reverse, 11, volt);

//   Robot.DriveFor(24, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Robot.DriveFor(-24, L_Settings);

//   Clamp.open();

//  // wait(150, msec);

//   Intake.spin(reverse, 10.5, volt);

// //  wait(150, msec);

//   Robot.TurnFor(-90, A60_Settings, 800);

//   Robot.DriveFor(20, L_Settings);

//   Linear.setMaxVoltages(6.5);

//   Robot.DriveFor(-25, L_Settings);

//   launch_task([&] {
//     fishMechLoop(200);
//   });

//   Robot.TurnFor(135, A120_Settings, 800);

//   FishMech.spin(reverse, 3, volt);

//   Robot.DriveFor(10, L_Settings);
// }