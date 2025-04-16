/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jkeep                                                     */
/*    Created:      1/1/2025, 7:18:58 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "v5lvgl.h"
#include "gif-pros-modified/gifclass.hpp"

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

motor ladyBrown = motor(PORT12, true);

motor_group LeftSide = motor_group(FrontLeft, BottomLeft, TopLeft);
motor_group RightSide = motor_group(FrontRight, BottomRight, TopRight);

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
  0.49, 
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
  0.62, 
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

  false,

  2,

  0, // -0.5

  1.f/1.f
);

// BACK TRACKER EXAMPLE:
Bucees::TrackingWheel BackTracker(
  PORT8,

  false,

  2.75,
  
  -1.5, // -2.5

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
  &AntiDrift,
  
  PORT3,

  PORT13,

  PORT7
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
bool togglePistakeDB;
bool togglePistake;
bool toggleGoalRushDB;
bool toggleGoalRush;
bool toggleAlignment = false;
bool toggleTipper = false;
bool allowForSpin = false;
bool startReset = false;
bool inAlignment = false;
bool ladyBrownMacro = false;
bool inStageMacro = false;

// 

// lady brown macro
const int lBStages = 2; // the amount of stages
const int timeOutTime = 1000; // change how long it has to reach the target
const int lBMotorPower = 12; // change the maximum speed
const int stopperDegrees = 600; // where to stop lb 
int currentStage = -1; // 
int targetStage = 0;
double stages[lBStages] = { // the stages and their degrees
  78,
  117.5 // 300 for normal
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

    float rotationPosition = ladyBrown.position(degrees);

    float motorPower = lBController.calculateMotorPower(target - rotationPosition);

    //std::cout << "rotPosition: " << rotationPosition << std::endl;
    //std::cout << "error: " << target - rotationPosition << std::endl;

    //åprintf("Rotation Position: %f \n", rotationPosition);

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

  launch_task([&] {lBPid(stopperDegrees, 1250, 9);});

  waitUntil(ladyBrownMacro == false);
  togglelBDB = false;
}

void toggleTipperF() {
  if (togglelBDB == true) return;
  inStageMacro = true;
  togglelBDB = true;

  toggleTipper = !toggleTipper;

  currentStage = -1;

  launch_task([&] {lBPid(625, 2000, 12);});

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

  launch_task([&] {lBPid(targetStage, 750);}); // start pid

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

void togglePistakeF() {
  if (togglePistakeDB == true) return;
  togglePistakeDB = true;

  pneumatics pistake(Brain.ThreeWirePort.H);

  togglePistake = !togglePistake;

  if (togglePistake == true) {
    pistake.open();
  } else {
    pistake.close();
  }


  wait(50, msec);
  togglePistakeDB  = false;
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

int activeTab = 0;
int currentAuton = 3;
bool elims = false;

lv_obj_t *tabview;
lv_obj_t *screen;
lv_disp_t *display;
lv_theme_t *th;
lv_obj_t *sw;
lv_obj_t *descBox;
lv_obj_t *descLabel;
lv_obj_t *AutonOneBtn;
lv_obj_t *AutonTwoBtn;
lv_obj_t *AutonThreeBtn;
lv_obj_t *AutonOneLabel;
lv_obj_t *AutonTwoLabel;
lv_obj_t *AutonThreeLabel;

void switchSkillsTab() {
  lv_obj_set_style_bg_color(AutonOneBtn, lv_color_hex(0x9E9E9E), LV_PART_MAIN);
  lv_obj_set_style_bg_color(AutonTwoBtn, lv_color_hex(0x9E9E9E), LV_PART_MAIN);
  lv_obj_set_style_bg_color(AutonThreeBtn, lv_color_hex(0x9E9E9E), LV_PART_MAIN);
  lv_label_set_text(AutonOneLabel, "Main Skills Run");
  lv_label_set_text(AutonTwoLabel, "Test Skills Run");
  lv_label_set_text(AutonThreeLabel, "Do Nothing");  
}

void switchBlueTab() {
  lv_obj_set_style_bg_color(AutonOneBtn, lv_color_hex(0x3088FF), LV_PART_MAIN);
  lv_obj_set_style_bg_color(AutonTwoBtn, lv_color_hex(0x3088FF), LV_PART_MAIN);
  lv_obj_set_style_bg_color(AutonThreeBtn, lv_color_hex(0x3088FF), LV_PART_MAIN);
  lv_label_set_text(AutonOneLabel, "Goal Rush");
  lv_label_set_text(AutonTwoLabel, "5 ring + Alliance Stake");
  lv_label_set_text(AutonThreeLabel, "Do Nothing");  
}

void switchRedTab() {
  lv_obj_set_style_bg_color(AutonOneBtn, lv_color_hex(0xFF3030), LV_PART_MAIN);
  lv_obj_set_style_bg_color(AutonTwoBtn, lv_color_hex(0xFF3030), LV_PART_MAIN);
  lv_obj_set_style_bg_color(AutonThreeBtn, lv_color_hex(0xFF3030), LV_PART_MAIN);
  lv_label_set_text(AutonOneLabel, "Goal Rush");
  lv_label_set_text(AutonTwoLabel, "5 ring + Alliance Stake");
  lv_label_set_text(AutonThreeLabel, "Do Nothing");
}

int tabWatcher() {
  activeTab = lv_tabview_get_tab_act(tabview);
 // if (lv_obj_has_state(sw, LV_STATE_CHECKED) == true) lv_obj_clear_state(sw, LV_STATE_CHECKED);

  while (1) {
    int currentTab = lv_tabview_get_tab_act(tabview);

    if (currentTab != activeTab) {
      activeTab = currentTab;

      if (activeTab == 0) {
        std::cout << "on red" << std::endl;
        switchRedTab();
      } else if (activeTab == 1) {
        std::cout << "on blue" << std::endl;
        switchBlueTab();
      } else {
        std::cout << "on skills" << std::endl;
        switchSkillsTab();
      }
    }

    wait(20, msec);
  }
}

void substring(char* str, const char* replacing, const char* replace) {
  char* pos = strstr(str, replacing);

  if (pos) {
    size_t previousLength = pos - str;
    size_t nextLength = strlen(pos + strlen(replacing));

    memmove(pos + strlen(replace), pos + strlen(replacing), nextLength + 1);

    memcpy(pos, replace, strlen(replace));
  }
}

static void sw_event_cb(lv_event_t * e)
{
    lv_obj_t * sw = lv_event_get_target(e);
    lv_obj_t * label = (lv_obj_t *)lv_event_get_user_data(e);
    char *text = lv_label_get_text(label);

    if (lv_obj_has_state(sw, LV_STATE_CHECKED)) {
      substring(text, "Elims?: No", "Elims?: Yes");
      elims = true;
      lv_label_set_text(label, text);
    } else {
      substring(text, "Elims?: Yes", "Elims?: No");
      elims = false;
      lv_label_set_text(label, text);
    }

}

void switchDescriptions(std::string side, int auton) {
  if (auton == 1) {
    if (side == "Red") { // red goal rush
      lv_label_set_text(descLabel, "Rings: 3 \nGoals: 2 \nAlliance Stake?: Yes \nTouches Ladder?: No\nElims?: No");
    } else if (side == "Blue") { // blue goal rush
      lv_label_set_text(descLabel, "Rings: 3 \nGoals: 2 \nAlliance Stake?: Yes \nTouches Ladder?: No\nElims?: No");
    } else {
      lv_label_set_text(descLabel, "59 Skills Run with hang");
    }
  } else if (auton == 2) {
    if (side == "Red") { // red 5 ring, alliance stake
      lv_label_set_text(descLabel, "Rings: 6 \nGoals: 1 \nAlliance Stake?: Yes \nTouches Ladder?: No\nElims?: No");
    } else if (side == "Blue") { // blue 5 ring, alliance stake
      lv_label_set_text(descLabel, "Rings: 6 \nGoals: 1 \nAlliance Stake?: Yes \nTouches Ladder?: Yes\nElims?: No");
    } else {
      lv_label_set_text(descLabel, "Test Run");
    }
  } else {
    lv_label_set_text(descLabel, "Does nothing.");
  }
}

static void btn_click_action(lv_event_t * event) {
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *obj = lv_event_get_target(event);
    int data = (int) lv_event_get_user_data(event);

    currentAuton = data;

    lv_obj_set_style_bg_color(obj, lv_color_darken(lv_obj_get_style_bg_color(obj, LV_PART_MAIN), 3.5), LV_PART_MAIN | LV_STATE_FOCUSED);

    if (code == LV_EVENT_CLICKED) {
      switch (activeTab) {
        case 0:
          switchDescriptions("Red", data);
          break;
        case 1:
          switchDescriptions("Blue", data);
          break;
        case 2:
          switchDescriptions("Skills", data);
          break;
      }
      printf("received integer of: %d\n", data);
    }

    return ;
}

void initAutonSelector() {

  screen = lv_scr_act();
  display = lv_disp_get_default();

  th = lv_theme_default_init(
    display,
    lv_palette_main(LV_PALETTE_LIME),
    lv_palette_main(LV_PALETTE_BLUE),
    LV_THEME_DEFAULT_DARK,
    LV_FONT_DEFAULT
  );

  lv_disp_set_theme(display, th);
  lv_obj_set_style_bg_color(screen, lv_palette_main(LV_PALETTE_NONE), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

  tabview = lv_tabview_create(screen, LV_DIR_TOP, 45);
  lv_obj_t *tab_btns = lv_tabview_get_tab_btns(tabview);

  lv_obj_set_style_radius(tab_btns, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabview, LV_OPA_TRANSP, LV_PART_MAIN);

  lv_obj_t *redSide = lv_tabview_add_tab(tabview, "Red Side");
  lv_obj_t *blueSide = lv_tabview_add_tab(tabview, "Blue Side");
  lv_obj_t *skills = lv_tabview_add_tab(tabview, "Skills");

  lv_obj_set_style_bg_color(tab_btns, lv_palette_darken(LV_PALETTE_GREY, 3), LV_PART_MAIN | LV_STATE_DEFAULT); // Set background color to white
  //lv_obj_set_style_bg_color(tab_btns, lv_color_hex(0xFFFFFF), LV_PART_ITEMS | LV_STATE_CHECKED);

  lv_obj_set_style_clip_corner(tab_btns, true, LV_PART_MAIN | LV_STATE_DEFAULT);

  static lv_style_t menuStyle;
  lv_style_init(&menuStyle);
  lv_style_set_radius(&menuStyle, 10);
  lv_style_set_bg_opa(&menuStyle, LV_OPA_COVER);
  lv_style_set_text_letter_space(&menuStyle, 3);
  lv_style_set_text_line_space(&menuStyle, 10);
  lv_style_set_pad_all(&menuStyle, 10);

  descBox = lv_obj_create(screen);
  lv_obj_add_style(descBox, &menuStyle, 0);
  lv_obj_set_size(descBox, 235, 175);
  lv_obj_center(descBox);
  lv_obj_set_pos(descBox, 100, 22.5);
  
  descLabel = lv_label_create(descBox);
  lv_label_set_text(descLabel, "No auton selected!");

  sw = lv_switch_create(descBox);
  lv_obj_set_align(sw, LV_ALIGN_BOTTOM_LEFT);
  //lv_obj_add_style(sw, LV_STATE_CHECKED);
  lv_obj_add_event_cb(sw, sw_event_cb, LV_EVENT_VALUE_CHANGED, descLabel);

  AutonOneBtn = lv_btn_create(screen); //create button, lv_scr_act() is deafult screen object
  lv_obj_add_event_cb(AutonOneBtn, btn_click_action, LV_EVENT_ALL, (void * ) 1); //set function to be called on button click
  lv_obj_add_style(AutonOneBtn, &menuStyle, 0);
  lv_obj_set_style_bg_color(AutonOneBtn, lv_color_hex(0xFF3030), LV_PART_MAIN);
  lv_obj_set_size(AutonOneBtn, 200, 50); //set the button size
  lv_obj_align(AutonOneBtn, LV_ALIGN_TOP_LEFT, 10, 60); //set the position to top mid

  AutonTwoBtn = lv_btn_create(screen); //create button, lv_scr_act() is deafult screen object
  lv_obj_add_event_cb(AutonTwoBtn, btn_click_action, LV_EVENT_ALL, (void * ) 2); //set function to be called on button click
  lv_obj_add_style(AutonTwoBtn, &menuStyle, 0);
  lv_obj_set_style_bg_color(AutonTwoBtn, lv_color_hex(0xFF3030), LV_PART_MAIN);
  lv_obj_set_size(AutonTwoBtn, 200, 50); //set the button size
  lv_obj_align(AutonTwoBtn, LV_ALIGN_TOP_LEFT, 10, 120); //set the position to top mid

  AutonThreeBtn = lv_btn_create(screen); //create button, lv_scr_act() is deafult screen object
  lv_obj_add_event_cb(AutonThreeBtn, btn_click_action, LV_EVENT_ALL,(void * ) 3); //set function to be called on button click
  lv_obj_add_style(AutonThreeBtn, &menuStyle, 0);
  lv_obj_set_style_bg_color(AutonThreeBtn, lv_color_hex(0xFF3030), LV_PART_MAIN);
  lv_obj_set_size(AutonThreeBtn, 200, 50); //set the button size
  lv_obj_align(AutonThreeBtn, LV_ALIGN_TOP_LEFT, 10, 180); //set the position to top mid

  AutonOneLabel = lv_label_create(AutonOneBtn); //create label and puts it inside of the button
  lv_label_set_text(AutonOneLabel, "AUTON 1"); //sets label text
  lv_obj_center(AutonOneLabel);

  AutonTwoLabel = lv_label_create(AutonTwoBtn); //create label and puts it inside of the button
  lv_label_set_text(AutonTwoLabel, "AUTON 2"); //sets label text
  lv_obj_center(AutonTwoLabel);

  AutonThreeLabel = lv_label_create(AutonThreeBtn); //create label and puts it inside of the button
  lv_label_set_text(AutonThreeLabel, "AUTON 3"); //sets label text
  lv_obj_center(AutonThreeLabel);

  launch_task([&] {tabWatcher();});

  //std::cout << "???" >> std::endl;
  switchRedTab();
}

void startScreen() {

  // static Gif gif("introScreen.gif", lv_scr_act());
  // waitUntil(gif.isFinished() == true);

  initAutonSelector();
}

void detectMotorDead() {
  while (1) {

    if (TopLeft.installed() == false || TopRight.installed() == false || BottomLeft.installed() == false || BottomRight.installed() == false || FrontLeft.installed() == false || FrontRight.installed() == false) {
      std::cout << "MOTOR UNINSTALLED" << std::endl;
    }

    wait(20, msec);
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
  ladyBrown.setBrake(hold);

  LeftSide.resetPosition();
  RightSide.resetPosition();

  lBController.setTimeoutTime(timeOutTime);
  lBController.setMaxVoltages(lBMotorPower);

  ladyBrown.resetPosition();

  launch_task([&] {startScreen();});
  launch_task([&] {detectMotorDead();});

  InertialSensor.calibrate();
  waitUntil(InertialSensor.isCalibrating() == false);

  RingFilter.setLightPower(100, pct);
  RingFilterBottom.setLightPower(100, pct);

  RingFilter.setLight(ledState::on);
  RingFilterBottom.setLight(ledState::on);

  Robot.initOdom();
  //Robot.initMCL({-59, -61}, {-1, 1}, {to_rad(87.5), to_rad(92.5)}, 150);
  //Robot.setRobotCoordinates({-55.6, -61.7, 79});
  //Robot.setRobotCoordinates({-64, 0, 90});
 // Robot.wallResetOdom();

  //Robot.wallResetOdom(0.1);

  //printCoordinates();


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
  std::map<int, std::map<int, std::function<void(bool)>>> autons = {
    {0, { // Red Side Autons
      {1, goalRushRed},
      {2, negSideRed},
      {3, doNothing}
    }},
    {1, { // Blue Side Autons
      {1, goalRushBlue},
      {2, negSideBlue},
      {3, soloBlueAWP}
    }},
    {2, { // Skills Autons
      {1, skills},
      {2, testSkills},
      {3, doNothing}
    }}
  };

  std::cout << activeTab << currentAuton << elims << std::endl;
  autons[activeTab][currentAuton](elims);

  //autons[0][1](false);
  return;
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

    float rotationPosition = ladyBrown.position(degrees);

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

void filterTest(COLOR_SORTER sortColor) {
  if (RingFilter.isNearObject() != true) return;
 // std::cout << "h" << std::endl;
  if (RingFilter.hue() < 15 && sortColor != FILTER_RED) return;
  //std::cout << "h1" << std::endl;
  if (RingFilter.hue() > 200 && sortColor != FILTER_BLUE) return;
  //std::cout << "h2" << std::endl;
  //if (RingFilter.hue() > 20 && RingFilter.hue() < 200) return;
  std::cout << "hey" << std::endl;
  Controller.rumble(".");
  wait(105, msec);
  Intake.spin(forward, 12, volt);
  wait(105, msec);
  Intake.spin(reverse, 12, volt);
}

void testFilterAgain() {
  filterTest(FILTER_BLUE);
}
bool dbReset = false;
bool attemptingToReset = false;

void resetLBF() {
  if (dbReset == true) return;
  dbReset = true;
  attemptingToReset = true;
  ladyBrown.spin(reverse, 6, volt);
  wait(1.5, sec);
  ladyBrown.stop();
  ladyBrown.setPosition(-38, deg);
  attemptingToReset = false;
  std::cout << "anoaoan" << std::endl;
  wait(0.2, sec);
  dbReset = false;
}

void testWallResetting() {
  Robot.wallResetOdom();
}

void usercontrol(void) {

  Controller.ButtonB.pressed(toggleClampF);
  Controller.ButtonA.pressed(toggleTipperF);
  Controller.ButtonY.pressed(toggleDoinkerF);
  Controller.ButtonRight.pressed(toggleGoalRushF);
  Controller.ButtonL1.pressed(togglelBF);
  Controller.ButtonUp.pressed(resetLBF);
  //Controller.ButtonR1.pressed(checkAlignment);
  Controller.ButtonR1.pressed(toggleStageMacro);
 //Controller.ButtonR1.released(checkAlignment2);
  Controller.ButtonDown.pressed(toggleAlignmentF);
  Controller.ButtonX.pressed(togglePistakeF);
  Controller.ButtonLeft.pressed(testWallResetting);
  //RingFilter.objectDetected(testFilterAgain);

  launch_task([&] {
    detectAlignment();
  });

  // launch_task([&] {
  //   while (1) {
  //     filterTest(FILTER_RED);
  //     wait(10, msec);
  //   };
  // });

  //std::cout << "1p" << std::endl;
  //Intake.spin(reverse, 12, volt);

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

    float rotationPosition =  ladyBrown.position(degrees);

    if (Controller.ButtonR1.pressing() && inStageMacro == false && attemptingToReset == false) {
      ladyBrown.spin(forward, 12, volt);
    } else if (rotationPosition < 1 && attemptingToReset == false) {
      ladyBrown.stop(hold);
    } else if ((Controller.ButtonR1.pressing() == false && ladyBrownMacro == false && inStageMacro == false && rotationPosition > 1 && attemptingToReset == false)) {
      std::cout << "spinning to bring back" << std::endl;
      ladyBrown.spin(reverse, 8.5, volt);
    }

    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
    // float sideArcLength = BackTracker.getDistanceTraveled();
    // float forwardArcLength = RightTracker.getDistanceTraveled();

    // float angularChange = to_rad(InertialSensor.rotation(rotationUnits::deg));

    // printf("sideArcR: %f\n", sideArcLength / angularChange);
    // printf("forwardArcR: %f\n", forwardArcLength / angularChange);

    printf("rot: %f \n", rotationPosition);

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
    Brain.Screen.printAt(50, 200, "Ladybrown Temp: %f", ladyBrown.temperature(temperatureUnits::fahrenheit));

    //Brain.Screen.drawImageFromFile("Brain_Screen_Logo.png", 0, 0);

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
  v5_lv_init();
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
