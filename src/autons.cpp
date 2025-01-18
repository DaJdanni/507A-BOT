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


void skills() {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  //-- Alliance Stake Score --//

  Intake.spin(reverse, 12, volt);

  wait(0.45, seconds);

  Intake.stop();

  FishMech.spin(forward, 1, volt);

  // launch_task([&] {
  //   fishMechLoop(150);
  // });

  //-- Mogo #1 Clamp --//

  Robot.DriveToPoint(0, 11, L_Settings, A0_Settings);

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

  Robot.DriveToPoint(-20, 36, L_Settings, A0_Settings); // 1st ring

  Robot.TurnFor(-15, A0_Settings, 350);

  Robot.DriveToPoint(-56, 59, L_Settings, A0_Settings); // 2nd ring

  wait(900, msec);

  Robot.TurnFor(150, A60_Settings, 500);

  Angular.setMaxVoltages(12);
  Linear.setMaxVoltages(9);

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(-46.5, 24.5, L_Settings, A0_Settings); // 3rd ring

  wait(600, msec);

 // Linear.setMaxVoltages(6);

  Robot.DriveToPoint(-47, 12.5, L_Settings, A0_Settings); // 4th ring

  wait(800, msec);

  Robot.DriveToPoint(-47, -3, L_Settings, A0_Settings); // 5th ring

  wait(750, msec);

  Robot.TurnFor(305, A60_Settings, 500);

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-65, 17, L_Settings, A0_Settings); // get 6th ring

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

  Robot.DriveToPoint(-50.814, 19, L_Settings, A0_Settings);

  Linear.setMaxVoltages(10);

  Robot.DriveToPoint(0, 19, L_Settings, A0_Settings);

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

  Robot.DriveToPoint(0, 26, L_Settings, A0_Settings);

  wait(350, msec);

  Robot.TurnFor(55, A0_Settings, 800);

  Robot.DriveFor(9, L_Settings, false, 850);

  Robot.DriveToPoint(20.5, 75, L_Settings, A0_Settings);

  wait(1000, msec);

  Robot.TurnFor(180, A120_Settings, 800);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(20, 12.5, L_Settings, A0_Settings); // 3rd

  wait(600, msec);

 // Linear.setMaxVoltages(2.5);

  Robot.DriveToPoint(20, -5, L_Settings, A0_Settings); // 4th

  wait(800, msec);

  Robot.DriveToPoint(20, -24, L_Settings, A0_Settings, 2000); // 5th

  wait(750, msec);

  Linear.setMaxVoltages(12);

  wait(1000, msec);

  Robot.TurnFor(60, A0_Settings, 1000);

  Robot.DriveFor(15, L_Settings, false, 1000); // 6th ring

  wait(500, msec);

  Robot.DriveFor(-10, L_Settings, false, 1000);

  Robot.TurnFor(-50, A0_Settings, 800);

  Robot.DriveFor(-15, L_Settings, true, 1500);

  Mogo.close(); 

  Intake.spin(forward, 12, volt); // mogo #2

  printCoordinates();

  Robot.DriveFor(17.5, L_Settings, false, 1000);

  Robot.TurnFor(0, A0_Settings, 1000);

  Robot.DriveFor(52.5, L_Settings);

  Intake.spin(reverse, 10.5, volt);

  printCoordinates();

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
 });

  Robot.DriveToPoint(-1, 65, L_Settings, A0_Settings); // 1st ring

  printCoordinates();

  wait(350, msec);

  //Robot.DriveFor(5, L_Settings, true);

  Robot.TurnFor(130, A120_Settings, 1000);

  Linear.setMaxVoltages(8);

  Robot.DriveFor(-43, L_Settings, true, 3000);

  Mogo.open(); // mogo #3

  Robot.TurnFor(-140, A120_Settings, 800);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveFor(31, L_Settings); // 2nd ring

  Robot.TurnFor(-90, A60_Settings, 1000);

  Robot.DriveFor(20, L_Settings); // 3rd ring

  Robot.TurnFor(0, A60_Settings, 1000);

  Robot.DriveFor(25, L_Settings); // 4th ring

  Linear.setMaxVoltages(7.5);

  Robot.DriveFor(-16, L_Settings);

  Robot.TurnFor(-90, A60_Settings, 1000);

  wait(500, msec);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(13, L_Settings);

  Robot.TurnFor(0, A60_Settings, 1000);

  Robot.DriveFor(18, L_Settings); // 5th ring

  wait(100, sec);

}

void disruptBlueRing(bool elims) {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  redFilter = true;
  waitTimeFilter = 150;
 // RingFilter.objectDetected(filter);

      launch_task([&] {
        fishMechLoop(215);
      });

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
    return;
  });

  //FishMech.spin(forward, 3, volt);

  Robot.DriveToPoint(0, 37.5, LOdom_Settings, A0_Settings);

  printCoordinates();

  Intake.spin(reverse, 10.5, volt);

  Robot.DriveToPoint(5, 45.5, LOdom_Settings, A0_Settings, 1000);

  printCoordinates();

  Robot.TurnFor(47.5, A0_Settings, 700);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(-24, LOdom_Settings, true, 1250);

  Mogo.open();

 // Intake.spin(reverse, 12, volt);

  wait(200, msec);

  Robot.TurnFor(90, A60_Settings, 500);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(10.5, 29, LOdom_Settings, A0_Settings);

  wait(400, msec);
  Robot.TurnFor(10.5, A0_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(15, L_Settings, false, 1000);

  wait(0.2, seconds);

  Linear.setMaxVoltages(6);

  Robot.DriveFor(-17, L_Settings, true, 1000);

  Robot.TurnFor(-183, A120_Settings, 1000);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(25, L_Settings, false, 1500);

  printCoordinates();

  wait(0.75, seconds);

  if (elims != true) {
      Robot.TurnFor(-45, A120_Settings, 1000);

      launch_task([&] {
        fishMechLoop(215);
      });

      Linear.setMaxVoltages(7);

      FishMech.spin(reverse, 3, volt);

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
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  waitTimeFilter = 170;
  blueFilter = true;
 // RingFilter.objectDetected(filter);

       launch_task([&] {
        fishMechLoop(215);
      });

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
    return;
  });

  FishMech.spin(forward, 3.5, volt);

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

  Mogo.open();

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

    FishMech.stop();

    launch_task([&] {
      fishMechLoop(215);
    });

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

void goalRushRed(bool elims) {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  Robot.DriveToPoint(0, -30, LOdom_Settings, A0_Settings, 0, true); //htbht
 // deactivateMotionChaining(true);
  Linear.setMaxVoltages(6.75);
  Robot.DriveToPoint(7.65, -49.5, L_Settings, A0_Settings, 0, true); // htbht
 printCoordinates();
 // Robot.DriveFor(-1.5, LOdom_Settings, true, 500); //htbht
  Linear.setMaxVoltages(8.5);

  //wait(100, sec);

  Mogo.open();

  wait(400, msec);

  Intake.spin(reverse, 12, volt);
 // printCoordinates();
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

  Robot.DriveFor(-30, L_Settings, true, 1500);

  //Robot.DriveToPoint(34, -28.5, LOdom_Settings, A0_Settings, 0, true); // htbht

  Mogo.open(); // Grab second goal

  //printCoordinates();

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

  if (elims != true) {
    Intake.spin(reverse, 12, volt);
    FishMech.spin(reverse, 3, volt);

    Robot.TurnFor(150, A120_Settings, 1000);

    Linear.setMaxVoltages(7);

    Robot.resetOdom();

    wait(0.1, sec);

    Robot.DriveFor(46.5, L_Settings, true, 20000);

    wait(100, sec);
  } else {

  }
}

void goalRushBlue(bool elims) {
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

  if (elims != true) {
    Intake.spin(reverse, 12, volt);

    Robot.TurnFor(-150, A120_Settings, 1000);

    Linear.setMaxVoltages(7.5);

    FishMech.spin(reverse, 3, volt);

    Robot.DriveFor(35, L_Settings, false, 10000);

    wait(100, sec);
  } else {

  }
}

void safeSoloRedAWP(bool elims) {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  FishMech.spin(forward, 3, volt);

  Robot.DriveFor(-28.5, L_Settings, true);

  Mogo.open();

  wait(150, msec);

  Intake.spin(reverse, 12, volt);

  wait(500, msec);

  Robot.TurnFor(-90, A60_Settings, 800);

  Robot.DriveFor(21.5, L_Settings, 1000);

  Linear.setMaxVoltages(7.5);

  Robot.DriveFor(-16, L_Settings);

  Robot.TurnFor(0, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(19.5, L_Settings);

  Robot.TurnFor(90, A60_Settings, 800);

  Mogo.close();

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true && RingFilter.hue() < 15);
    Intake.stop();
  });

  Linear.setMaxVoltages(8.5);

  Robot.DriveFor(27, L_Settings);

  Linear.setMaxVoltages(12);

  Intake.spin(reverse, 10.5, volt);

  Robot.DriveFor(24, L_Settings);

  Robot.TurnFor(0, A60_Settings, 800);

  Robot.DriveFor(-21, L_Settings);

  Mogo.open();

 // wait(150, msec);

  Intake.spin(reverse, 11, volt);

//  wait(150, msec);

  Robot.TurnFor(90, A60_Settings, 800);

  Robot.DriveFor(21, L_Settings);

  Linear.setMaxVoltages(7.5);

  Robot.DriveFor(-27.5, L_Settings);

  launch_task([&] {
    fishMechLoop(200);
  });

  Robot.TurnFor(-135, A120_Settings, 800);

  FishMech.spin(reverse, 3, volt);

  Robot.DriveFor(10, L_Settings);  
}

void safeSoloBlueAWP(bool elims) {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  Robot.DriveFor(-28.5, L_Settings, true);

  Mogo.open();

  wait(150, msec);

  Intake.spin(reverse, 12, volt);

  wait(500, msec);

  Robot.TurnFor(90, A60_Settings, 800);

  Robot.DriveFor(21.5, L_Settings);

  Linear.setMaxVoltages(6);

  Robot.DriveFor(-16, L_Settings);

  Robot.TurnFor(0, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(19.5, L_Settings);

  Robot.TurnFor(-90, A60_Settings, 800);

  Mogo.close();

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true && RingFilter.hue() > 100);
    Intake.stop();
  });

  Linear.setMaxVoltages(8.5);

  Robot.DriveFor(24, L_Settings);

  Linear.setMaxVoltages(12);

  Intake.spin(reverse, 11, volt);

  Robot.DriveFor(24, L_Settings);

  Robot.TurnFor(0, A60_Settings, 800);

  Robot.DriveFor(-24, L_Settings);

  Mogo.open();

 // wait(150, msec);

  Intake.spin(reverse, 10.5, volt);

//  wait(150, msec);

  Robot.TurnFor(-90, A60_Settings, 800);

  Robot.DriveFor(20, L_Settings);

  Linear.setMaxVoltages(6.5);

  Robot.DriveFor(-25, L_Settings);

  launch_task([&] {
    fishMechLoop(200);
  });

  Robot.TurnFor(135, A120_Settings, 800);

  FishMech.spin(reverse, 3, volt);

  Robot.DriveFor(10, L_Settings);
}