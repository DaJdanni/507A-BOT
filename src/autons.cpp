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

void bottomFilter(int timeout, vex::brakeType braketype = brakeType::coast) {
  int currentTime = 0;
  while (1) {

    if (RingFilterBottom.isNearObject() == true) {
      std::cout << "DETECTED" << std::endl;
      Intake.stop(braketype);
      break;        
    }

    currentTime += 10;

    if (currentTime >= timeout) break;

    wait(10, msec);
  }
}

void topFilter(int timeout, vex::brakeType braketype = brakeType::coast) {
  int currentTime = 0;
  while (1) {

    if (RingFilter.isNearObject() == true) {
      std::cout << "DETECTED" << std::endl;
      Intake.stop(braketype);
      break;        
    }

    currentTime += 10;

    if (currentTime >= timeout) break;

    wait(10, msec);
  }
}


void filter(COLOR_SORTER sortColor) {
  // red ring hue = 15
  // blue ring hue = 210
  if (RingFilter.isNearObject() != true) return;
 // std::cout << "h" << std::endl;
  if (RingFilter.hue() < 15 && sortColor != FILTER_RED) return;
  //std::cout << "h1" << std::endl;
  if (RingFilter.hue() > 210 && sortColor != FILTER_BLUE) return;
  //std::cout << "h2" << std::endl;
  if (RingFilter.hue() > 20 && RingFilter.hue() < 200) return;
  std::cout << "hey" << std::endl;
  Controller.rumble(".");
  wait(115, msec);
  Intake.stop(coast);
  wait(350, msec);
  Intake.spin(reverse, 12, volt);
}


//   printCoordinates();
void startDrivetrain(double speed) {
  LeftSide.spin(fwd, speed, volt);
  RightSide.spin(fwd, speed, volt);
}

void stopDrivetrain(brakeType brake) {
  LeftSide.stop(brake);
  RightSide.stop(brake);
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

double stage1Macro = 90;

void skills() {
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  ladyBrown1.resetPosition();
  ladyBrown2.resetPosition();

  launch_task([&] {
    limitLadyBrown();
  });

  set_intake(12);

  wait(400, msec); // wait until finished scoring red alliance stake

  // GOAL #1 SECTION //

  Robot.DriveToPoint(0, 14, L_Settings, A0_Settings);

  set_intake(0);

  Robot.TurnFor(-90, A60_Settings, 800);

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(28.5, 14, L_Settings, A0_Settings, 0, true);

  Clamp.open();

  wait(200, msec);

  Linear.setMaxVoltages(12);

  set_intake(12);

  Robot.TurnFor(0, A60_Settings, 800);

  // RING 1/2 FOR GOAL #1 SECTION //

  activateMotionChaining(false, 6.5);

  Robot.DriveToPoint(28.5, 29.5, L_Settings, A0_Settings); // ring 1

  deactivateMotionChaining(false);

  Robot.DriveToPoint(48, 90, L_Settings, A0_Settings); // ring 2

  // WALL STAKE #1 SECTION //

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(44, 61.5, L_Settings, A0_Settings, 0, true); // get in position

  Robot.TurnFor(90, A60_Settings, 800); // face wall stake

  launch_task([&] {lBPid(95);});

  detectJams = false;

  Linear.setMaxVoltages(7.5);
  Robot.DriveFor(9999, L_Settings, true, 3500, true);  // drive infinite for 3.5 seconds

  wait(1750, msec);

  set_intake(0); // stop intaking so we can score

  wait(250, msec);

  lBPid(600); // score

  launch_task([&] {lBPid(0);}); // reset lady brown

  Robot.waitChassis(); // wait for the time to be up (3.5 second time)

  // RINGS 3-6 FOR GOAL #1 SECTION //

  detectJams = true; // detect jams again since we're done with LB

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(44, 61.75, L_Settings, A0_Settings, 0, true); // back up

  Robot.TurnFor(180, L_Settings, 800); // face the rings 

  set_intake(12);

  Linear.setMaxVoltages(7.5);

  Robot.DriveToPoint(45.5, 0, L_Settings, A0_Settings); // drive straight into rings 3,4,5

  wait(800, msec);

  Robot.TurnFor(35, L_Settings, 800); // adjust position to face 6th ring

  Robot.DriveToPoint(60, 19, L_Settings, A0_Settings); // grab it

  wait(500, msec);

  Robot.DriveToPoint(70, -5, L_Settings, A60_Settings, 2000, true); // go into the corner

  Clamp.close();
  set_intake(-12); // prevent goal from getting dragged

  // GOAL #2 CORNER SECTION //

  activateMotionChaining(false, 6.5);

  Robot.DriveToPoint(57, 62, L_Settings, A0_Settings);

  deactivateMotionChaining();

  set_intake(12); // start intaking when we get close to the ring

  Robot.DriveToPoint(48, 111, L_Settings, A0_Settings);  // grab ring

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

  ladyBrown1.setPosition(0, deg);
  ladyBrown2.setPosition(0, deg);
  ladyBrown.setPosition(0, deg);


  launch_task([&] {
    lBPid(140);
    launch_task([&] {topFilter(5000);});
  });

  launch_task([&] {
  waitUntil(GoalDetector.objectRawSize() == -1 && GoalDetector.objectDistance(inches) < 1.5);
  goalRush.open();
  std::cout << "clamp" << std::endl;
  });

  Intake.spin(reverse, 12, volt);

  Doinker.open();

  Robot.DriveToPoint(-1, 38.5, L_Settings, A60_Settings);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 13, L_Settings, A0_Settings, 1500, true, true);

  // wait(200, msec);
  // Intake.stop();
  wait(550, msec);;
  goalRush.close();
  wait(100, msec);
  Robot.DriveFor(-4, L_Settings, false, 300);
  Doinker.close();
  Robot.waitChassis();

  //wait(500, msec);

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(-7, 40, L_Settings, A0_Settings, 2500, true);
  Linear.setMaxVoltages(12);

  Clamp.open();

  //wait(200, sec);

  wait(200, msec);
  
  Intake.spin(reverse, 12, volt);

  printCoordinates();

  //Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(22, 5, L_Settings, A0_Settings, 0, false, true);

  wait(800, msec);
  
  Clamp.close();

  Robot.waitChassis();

  //launch_task([&] {lBPid(72.5);});//63.5 84

  printCoordinates();

  Robot.TurnFor(115, A0_Settings, 800);
  //wait(1000, sec);
  Linear.setMaxVoltages(12);

  //wait(150, msec);

  //set_intake(12);
  Intake.spin(reverse, 12, volt);

  wait(150, msec);

  startDrivetrain(5.5);

  launch_task([&] {topFilter(5000);});

  wait(1500, msec);

  LeftSide.stop();
  RightSide.stop();

  Linear.setMaxVoltages(12);

  //Robot.DriveFor(-8, L_Settings, false, 650);

  //Robot.TurnFor(65, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  printCoordinates();

  //Robot.DriveFor(8, L_Settings, false, 200);
  
  Robot.DriveToPoint(-30, -16, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(154, A0_Settings, 800);

  Robot.DriveFor(15, L_Settings, false, 1000);
  wait(150, msec);
  Robot.DriveFor(-8, L_Settings, false, 500);

  ladyBrown.spin(forward, 12, volt);
  wait(400, msec);
  ladyBrown.stop(hold);
 
  Linear.setMaxVoltages(10.5);
  launch_task([&] {
    lBPid(-10);
    ladyBrown1.setPosition(0, deg);
    ladyBrown2.setPosition(0, deg);
    ladyBrown.setPosition(0, deg);
  });
  Robot.DriveToPoint(-24, 20, L_Settings, A0_Settings, 0, true);

  Clamp.open();

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  printCoordinates(false);
}

void goalRushBlue(bool elims) {

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  ladyBrown1.setPosition(0, deg);
  ladyBrown2.setPosition(0, deg);
  ladyBrown.setPosition(0, deg);

  // launch_task([&] {
  //   lBPid(67);
  //   wait(200, msec);
  //   lBPid(190);
  // });

  launch_task([&] {
  waitUntil(GoalDetector.objectRawSize() == -1 && GoalDetector.objectDistance(inches) < 1.5);
  goalRush.open();
  std::cout << "clamp" << std::endl;
  });

  detectJams = true;

  Intake.spin(reverse, 12, volt);

  launch_task([&] {topFilter(5000);});

  Doinker.open();

  //Linear.setMaxVoltages(11);
  Robot.DriveToPoint(-1.5, 38.5, L_Settings, A60_Settings);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 13, L_Settings, A0_Settings, 1500, true, true);

  // wait(200, msec);
  // Intake.stop();
  wait(550, msec);;
  goalRush.close();
  wait(100, msec);
  Robot.DriveFor(-4, L_Settings, false, 300);
  Doinker.close();
  Robot.waitChassis();

  //wait(500, msec);

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(-7, 39, L_Settings, A0_Settings, 2000, true);
  Linear.setMaxVoltages(12);

  Clamp.open();

  wait(200, msec);
  
  Intake.spin(reverse, 12, volt);

  //Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(3, 3.5, L_Settings, A0_Settings);

  launch_task([&] {lBPid(72.5);});//63.5 84

  printCoordinates();

  Robot.TurnFor(Robot.getAbsoluteHeading() + 20, A0_Settings, 800);
  //wait(1000, sec);
 // Robot.DriveFor(-8, L_Settings, false, 650);

  //wait(150, msec);

  Clamp.close();

  //set_intake(12);
  Intake.spin(reverse, 12, volt);

  Robot.DriveFor(6.5, L_Settings, true, 750);

  wait(550, msec);
  Intake.stop(coast);
  wait(100, msec);
  lBPid(150, 400);
  //wait(250, msec);

  startDrivetrain(6);
  Intake.spin(reverse, 12, volt);

  launch_task([&] {topFilter(3000);});

  wait(800, msec);

  LeftSide.stop();
  RightSide.stop();

  Linear.setMaxVoltages(12);

  //Robot.DriveFor(-8, L_Settings, false, 650);

  //Robot.TurnFor(65, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  printCoordinates();

  //Robot.DriveFor(8, L_Settings, false, 200);

  Robot.DriveToPoint(54.5, 23, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(160, A0_Settings, 800);

  Robot.DriveFor(15, L_Settings, false, 1000);
  wait(150, msec);
  Robot.DriveFor(-8, L_Settings, false, 500);

  ladyBrown.spin(forward, 12, volt);
  wait(400, msec);
  ladyBrown.stop(hold);
 
  Linear.setMaxVoltages(10.5);
  launch_task([&] {
    lBPid(-10);
    ladyBrown1.setPosition(0, deg);
    ladyBrown2.setPosition(0, deg);
    ladyBrown.setPosition(0, deg);
  });
  Robot.DriveToPoint(17.5, 44.5, L_Settings, A0_Settings, 0, true);

  Clamp.open();

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  printCoordinates(false);

}

void negSideBlue(bool elims) {
  // hey incase im not here, the setup is just parallel to the line tape 
  // also dont touch skills unless ur a sigma
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  Robot.setRobotCoordinates({0, 0, 31});

  //wait(100, sec);

  Robot.DriveFor(6.5, L_Settings, 600);

  ladyBrown.spin(forward, 12, volt);
  wait(400, msec);
  ladyBrown.stop(hold);

  Linear.setMaxVoltages(9);
  launch_task([&] {
    wait(500, msec); 
    lBPid(-50, 3000); 
    ladyBrown1.setPosition(0, deg);
    ladyBrown2.setPosition(0, deg);
    ladyBrown.setPosition(0, deg);
  });
  Robot.DriveToPoint(-15, -28.5, L_Settings, A0_Settings, 0, true);

  Clamp.open();

  wait(200, msec);

  Robot.TurnFor(-147, A120_Settings, 500);

  Intake.spin(reverse, 12, volt);

  wait(200, msec);
  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-29, -41, L_Settings, A0_Settings);

  wait(350, msec);

  Robot.DriveToPoint(-15, -26, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(-40, -27, L_Settings, A0_Settings);

  wait(200, msec);

  Robot.TurnFor(-200, A60_Settings, 500);

  Robot.DriveToPoint(-44, -41.5, L_Settings, A0_Settings);

  printCoordinates();

  Linear.setMaxVoltages(7.5);

  Robot.DriveToPoint(-44, 7, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(12);

  Robot.TurnFor(277, A120_Settings);

  wait(200, sec);

  printCoordinates();

  startDrivetrain(4.5);

  wait(1500, msec);

  startDrivetrain(-4.5);

  wait(750, msec);

  startDrivetrain(4.5);
}

void negSideRed(bool elims) {
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  Robot.setRobotCoordinates({0, 0, -48});
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