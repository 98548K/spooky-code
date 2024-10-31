//Library configuration
#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern drivetrain Drivetrain;
extern digital_out Clamp;
extern digital_out PTO;
extern digital_out descore_mech;
extern motor intake;
extern digital_out elevation;
extern inertial Inertial1;


void  vexcodeInit( void );
// A global instance of competition
competition Competition;

//robot config.cpp stuff
controller Controller1 = controller(primary);
brain  Brain;
//Drivetrain Configuration
motor LF = motor(PORT8, ratio18_1, true);
motor LM = motor(PORT10, ratio18_1, true);
motor LB = motor(PORT2, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LF, LM, LB);
motor RF = motor(PORT3, ratio18_1, false);
motor RM = motor(PORT15, ratio18_1, false);
motor RB = motor(PORT1, ratio18_1, false);
motor_group RightDriveSmart = motor_group(RF, RM, RB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
//Sensors
inertial Inertial1 = inertial(PORT7);
rotation tracking1 = rotation(PORT20);
rotation tracking2 = rotation(PORT19);
rotation claw_tracking = rotation(PORT18);
optical Optical1 = optical(PORT17);
//Motors
motor intake = motor(PORT9, ratio6_1, false);
motor claw_mech = motor(PORT6, ratio6_1,false);
// Pistons
digital_out Clamp = digital_out(Brain.ThreeWirePort.H);
digital_out PTO = digital_out(Brain.ThreeWirePort.B);
digital_out descore_mech = digital_out(Brain.ThreeWirePort.E);
digital_out elevation = digital_out(Brain.ThreeWirePort.F);

//Function for optical sensor to outtake blue
void blue_detected() {
  if (Optical1.color() == blue){
    intake.spin(reverse);
  }
  else if (Optical1.color() == ClrSkyBlue){
    intake.spin(reverse);
  }
  else if (Optical1.color() == ClrLightBlue){
    intake.spin(reverse);
  }
}
//Function for optical sensor to outtake blue
void red_detected() {
  if (Optical1.color() == red){
    intake.spin(reverse);
  }
}
//Prank that doesn't let josh intake rings. reserved for april fools day 
void funny_prank() {
  Optical1.setLightPower(100,pct);
  Optical1.objectDetected(blue_detected);
  Optical1.objectDetected(red_detected);
  wait (3,sec);
}
      
//Function for PID turning
void PID_turn(double LeftVelocity,double RightVelocity,double inches_traveled) {
  LeftDriveSmart.setVelocity(LeftVelocity, pct);
  RightDriveSmart.setVelocity(RightVelocity,pct);
  Drivetrain.driveFor(inches_traveled, inches,false);
  wait(1,sec);
  Drivetrain.stop();
}

//Function for resetting both sides of the drivetrain to the same velocity
void Reset_Both_Sides(double same_velocity) {
  LeftDriveSmart.setVelocity(same_velocity, pct);
  RightDriveSmart.setVelocity(same_velocity,pct);
}
//finished as of 10/15
void right_red() {
  Optical1.objectDetected(blue_detected);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-15,inches);
  Reset_Both_Sides(20);
  Drivetrain.driveFor(-4,inches);
  Clamp.set(true);
  Drivetrain.driveFor(0,inches,false);
  intake.spin(forward);
  Clamp.set(true);
  Drivetrain.turnFor(-55,degrees);
  Drivetrain.driveFor(12,inches);
  Optical1.objectDetected(blue_detected);
  wait (4,sec);
  PID_turn(0,100,-3);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-16,inches);
}
//finished as of 10/15
void left_red() {
  Optical1.setLightPower(100,pct);
  Optical1.objectDetected(blue_detected);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-15,inches);
  Reset_Both_Sides(20);
  Drivetrain.driveFor(-4,inches);
  Clamp.set(true);
  wait (1,sec);
  intake.spin(forward);
  Drivetrain.turnFor(58,degrees);
  Optical1.objectDetected(blue_detected);
  Drivetrain.driveFor(10,inches);
  Optical1.objectDetected(blue_detected);
  wait (1,sec);
  Optical1.objectDetected(blue_detected);
  intake.spin(forward);
  Drivetrain.turnFor(53,degrees);
  Drivetrain.driveFor(6,inches);
  wait (1,sec);
  Drivetrain.driveFor(-11,inches);
  Drivetrain.turnFor(-9,degrees);
  Drivetrain.driveFor(13,inches);
  PID_turn(100,0,-10);
  Reset_Both_Sides(100);
  wait (2,sec);
  Drivetrain.driveFor(-10,inches);
}
//finished as of 10/16
void right_blue() {
  Optical1.objectDetected(red_detected);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-15,inches);
  Reset_Both_Sides(20);
  Drivetrain.driveFor(-4,inches);
  Clamp.set(true);
  wait(1,sec);
  intake.spin(forward);
  Drivetrain.turnFor(-60,degrees);
  Drivetrain.driveFor(8,inches);
  Drivetrain.turnFor(-44,degrees);
  intake.spin(forward);
  Drivetrain.driveFor(7,inches);
  Drivetrain.driveFor(-13,inches);
  Drivetrain.turnFor(10,degrees);
  Drivetrain.driveFor(15,inches);
}
//finished as of 10/17
void skills_auton() {
  Optical1.objectDetected(blue_detected);
  Inertial1.calibrate();
  Reset_Both_Sides(30);
  intake.spinFor(1500,degrees);
  Drivetrain.driveFor(8,inches);
  Drivetrain.turnFor(55,degrees);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(-10,inches);
  Drivetrain.driveFor(-1,inches,false);
  Clamp.set(true);
  Reset_Both_Sides(50);
  Drivetrain.turnFor(-62,degrees);
  //delay to avoid a wheely
  wait(0.05,sec);
  Reset_Both_Sides(40);
  intake.spin(forward);
  Drivetrain.driveFor(10.9,inches);
  //this is the most inconsitent value
  PID_turn(0,50,10.5);
  //
  Reset_Both_Sides(50);
  wait (1,sec);
  Drivetrain.driveFor(9.5,inches);
  wait (1,sec);
  Drivetrain.turnFor(-49,degrees);
  Drivetrain.driveFor(10.8,inches);
  wait (2,sec);
  Drivetrain.driveFor(6,inches);
  wait (1,sec);
  //
  Reset_Both_Sides(40);
  Drivetrain.driveFor(-5.5,inches);
  Drivetrain.turnFor(66,degrees);
  Drivetrain.driveFor(5,inches);
  Reset_Both_Sides(30);
  wait (1,sec);
  Drivetrain.turnFor(50,degrees);
  Reset_Both_Sides(40);
  intake.spin(forward);
  Drivetrain.driveFor(26,inches);
  Reset_Both_Sides(30);
  wait (3,sec);
  Drivetrain.driveFor(-33,inches);
  Drivetrain.turnFor(25,degrees);
  Drivetrain.driveFor(-3,inches);
  Clamp.set(false);
  intake.stop();
  Drivetrain.driveFor(9,inches);
  Drivetrain.turnFor(-83,degrees);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(10.5,inches,false);
  wait (2,sec);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-38,inches);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(-7,inches);
  //other side
  Clamp.set(true);
  Reset_Both_Sides(50);
  Drivetrain.turnFor(62,degrees);
  //delay to avoid a wheely
  wait(0.05,sec);
  Reset_Both_Sides(40);
  intake.spin(forward);
  Drivetrain.driveFor(10.5,inches);
  //this is the most inconsitent value
  PID_turn(50,0,10.5);
  //
  Reset_Both_Sides(50);
  wait (1,sec);
  Drivetrain.driveFor(9.5,inches);
  wait (1,sec);
  Drivetrain.turnFor(52,degrees);
  Drivetrain.driveFor(10.8,inches);
  wait (2,sec);
  Drivetrain.driveFor(6,inches);
  wait (1,sec);
  //
  Reset_Both_Sides(40);
  Drivetrain.driveFor(-4,inches);
  Drivetrain.turnFor(-63,degrees);
  Drivetrain.driveFor(5,inches);
  Drivetrain.turnFor(-70,degrees);
  Drivetrain.driveFor(-7,inches);
  Clamp.set(false);
  PID_turn(100,0,2);
  Drivetrain.driveFor(20,inches);
}
//finished as of 10/15
void left_blue() {
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-15,inches);
  Reset_Both_Sides(20);
  Drivetrain.driveFor(-4,inches);
  Clamp.set(true);
  Drivetrain.driveFor(0,inches,false);
  intake.spin(forward);
  Clamp.set(true);
  Drivetrain.turnFor(55,degrees);
  Drivetrain.driveFor(12,inches);
  wait (4, sec);
  PID_turn(100,0,-3);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-16,inches);
}
//finished as of 10/16
void right_blue_elims() {
  right_blue();
}
//finished as of 10/22
void right_red_elims() {
  claw_mech.setVelocity(100,pct);
  Optical1.setLightPower(100,pct);
  Optical1.objectDetected(blue_detected);
  Reset_Both_Sides(70);
  Drivetrain.driveFor(25,inches);
  wait (1,sec);
  Drivetrain.turnFor(30,degrees);
  claw_mech.spinFor(900,degrees);
  wait (1,sec);
  claw_mech.spinFor(-1000,degrees);
  Drivetrain.turnFor(15,degrees);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-22,inches);
  Reset_Both_Sides(70);
  Drivetrain.driveFor(-3,inches);
  Clamp.set(true);
  Drivetrain.driveFor(3,inches);
  Drivetrain.turnFor(22,degrees);
  intake.spin(forward);
  Drivetrain.driveFor(13,inches);
  wait (1,sec);
  Optical1.setLightPower(100,pct);
  Optical1.objectDetected(blue_detected);
  Drivetrain.setDriveVelocity(20, pct);
}
//finished as of 10/15
void left_red_elims() {
  left_red();
}
//finished as of 10/22
void left_blue_elims() {
  claw_mech.setVelocity(100,pct);
  Optical1.setLightPower(100,pct);
  Optical1.objectDetected(red_detected);
  Reset_Both_Sides(70);
  Drivetrain.driveFor(25,inches);
  Drivetrain.turnFor(-40,degrees);
  claw_mech.spinFor(900,degrees);
  wait (1,sec);
  claw_mech.spinFor(-1000,degrees);
  Drivetrain.turnFor(-7,degrees);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-22,inches);
  Reset_Both_Sides(70);
  Drivetrain.driveFor(-3,inches);
  Clamp.set(true);
  Drivetrain.driveFor(3,inches);
  Drivetrain.turnFor(-20,degrees);
  intake.spin(forward);
  Drivetrain.driveFor(13,inches);
}
// VEXcode generated functions
// define variable for remote controller enable/disable 
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  bool Controller1RightShoulderControlMotorsStopped = true;
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      if (Controller1.ButtonR1.pressing()) {
          while (claw_tracking.position(degrees) <= 70){
            claw_mech.spin(forward);
          }
      wait (.1,sec);
      claw_mech.spin(forward);
      claw_mech.stop(hold);
      Controller1RightShoulderControlMotorsStopped = false;
      }
       else if (Controller1.ButtonR2.pressing()) {
        claw_mech.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;

      } else if (!Controller1RightShoulderControlMotorsStopped) {
        claw_mech.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }

      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();

      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}


//robot config.cpp stuff



bool Clamping = false;
bool elevate = false;
bool tier_1 = false;

//settings:
double kp = 0.0;
double ki = 0.0;
double kd = 0.0;
double turnkp = 0.0;
double turnki = 0.0;
double turnkd = 0.0;

//autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

int Error; 
//sensor value - desired value : position
int PrevError = 0; 
//position 20 miliseconds ago
int Derivative; 
//Error - PrevError : speed
int TotalError; 
//TotalError = TotalError + Error

int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError;

bool ResetDriveSensors = true;

//varibles modified for use:
bool enablePID_Drive = true;
int PID_Drive (){

while (enablePID_Drive) {
  if (ResetDriveSensors) {
    ResetDriveSensors = false;
    LeftDriveSmart.setPosition(0,degrees);
    RightDriveSmart.setPosition(0,degrees);
  }

  //get the position of both motors
  int leftMotorPosition = LeftDriveSmart.position(degrees);
  int rightMotorPosition = RightDriveSmart.position(degrees);
  

  ////////////////////////////////////////////////////////////////////////////////
  //lateral movement PID
  ///////////////////////////////////////////////////////////////////////////////

  //get average of the two motors
  int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    Error = averagePosition - desiredValue;

    //derivative
    Error = Error - PrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    TotalError += Error;

    double lateralMotorPower = Error * kp + Derivative * kd + TotalError * ki;

   ////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////////////////////////////////////////////
  //turning movement PID
  ///////////////////////////////////////////////////////////////////////////////
  //get average of the two motors
  int turnDifference = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    turnError = turnDifference - desiredTurnValue;

    //derivative
    turnDerivative = turnError - turnPrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    //turnTotalError += turnError

    double turnMotorPower = turnError * turnkp + turnDerivative * turnkd;


  ////////////////////////////////////////////////////////////////////////////////

    LeftDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);
    RightDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);





  PrevError = Error;
  turnPrevError = Error;
}

return 1;
}



int auton = 0;
bool draw = true;
//Menu for left side autons
void Menu1() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(0, 0, 250,117);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(0, 117, 250,117);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(250, 0, 250,117);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(250, 117, 250,117);
  Brain.Screen.setCursor(3,3);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Left Blue");
  Brain.Screen.setCursor(9,3);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Left Red");
  Brain.Screen.setCursor(3,33);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Left Blue Elims");
  Brain.Screen.setCursor(9,33);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Left Red Elims");
  //touch screen
  //left side menu
  while(true) {
    Brain.Screen.setFillColor(blue);
      if (Brain.Screen.xPosition()< 250){
      if (Brain.Screen.yPosition()< 140){
        auton = 2;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Left Blue      ");
      } 
      }
      if (Brain.Screen.xPosition()< 250){
      if (Brain.Screen.yPosition()> 120){
        auton = 3;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Left Red       ");
      }
      } 
      if (Brain.Screen.xPosition()> 240){
      if (Brain.Screen.yPosition()< 140){
        auton = 4;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Left Blue Elims");
      }
      } 
      if (Brain.Screen.xPosition()> 240){
      if (Brain.Screen.yPosition()> 120){
        auton = 5;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Left Red Elims ");
      }
      } 
      wait(20, msec);
  

      // Back button code
      if (Controller1.ButtonX.pressing()){
        Brain.Screen.clearScreen();
        draw = true;
        wait(0.1, sec);
        return;
      }
      wait(20, msec);
  }
}
//Menu for right side autons
void Menu2() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(0, 0, 250,117);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(0, 117, 250,117);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(250, 0, 250,117);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(250, 117, 250,117);
  Brain.Screen.setCursor(3,3);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Right Blue");
  Brain.Screen.setCursor(9,3);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Right Red");
  Brain.Screen.setCursor(3,33);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Right Blue Elims");
  Brain.Screen.setCursor(9,33);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Right Red Elims");
  wait(0.5, sec);
  //touch screen
  //right side menu
    while(true) {
    Brain.Screen.setFillColor(blue);
      if (Brain.Screen.xPosition()< 250){
      if (Brain.Screen.yPosition()< 140){
        auton = 6;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Right Blue      ");
      } 
      }
      if (Brain.Screen.xPosition()< 250){
      if (Brain.Screen.yPosition()> 120){
        auton = 7;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Right Red       ");
      }
      } 
      if (Brain.Screen.xPosition()> 240){
      if (Brain.Screen.yPosition()< 140){
        auton = 8;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Right Blue Elims");
      }
      } 
      if (Brain.Screen.xPosition()> 240){
      if (Brain.Screen.yPosition()> 120){
        auton = 9;
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Right Red Elims ");
      }
      } 
      wait(20, msec);

      // Back button code
      if (Controller1.ButtonX.pressing()){
        Brain.Screen.clearScreen();
        draw = true;
        wait(0.1, sec);
        return;
      }
      wait(20, msec);
    }
  }
  
  
  

void initial_menu() {
  while (true){
    if (draw == true){
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(white);
      Brain.Screen.drawRectangle(0, 0, 180,240);
      Brain.Screen.setFillColor(black);
      Brain.Screen.drawRectangle(300, 0, 180,240);
      Brain.Screen.setFillColor(ClrPurple);
      Brain.Screen.drawRectangle(180, 0, 120,120);
      Brain.Screen.setFillColor(ClrPink);
      Brain.Screen.drawRectangle(180, 120, 120,120);
      Brain.Screen.setCursor(6,9);
      Brain.Screen.setFillColor(white);
      Brain.Screen.setPenColor(black);
      Brain.Screen.print("Left");
      Brain.Screen.setCursor(6,39);
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.print("Right");
      Brain.Screen.setCursor(3,22);
      Brain.Screen.setFillColor(ClrPurple);
      Brain.Screen.setPenColor(white);
      Brain.Screen.print("Skills");
      Brain.Screen.setCursor(9,20);
      Brain.Screen.setFillColor(ClrPink);
      Brain.Screen.setPenColor(white);
      Brain.Screen.print("Calibrate");
      draw = false;
    }
  //touch screen
    //left side menu
    Brain.Screen.setFillColor(white);
    if (Brain.Screen.pressing()){
     if (Brain.Screen.xPosition()< 180){
      Menu1();
     } 
     //Calibrate
     if (Brain.Screen.xPosition()> 180){
       if (Brain.Screen.xPosition()< 300){
        if (Brain.Screen.yPosition()> 120){
          Inertial1.calibrate();
          Brain.Screen.setCursor(1,1);
          Brain.Screen.setFillColor(white);
          Brain.Screen.setPenColor(black);
          Brain.Screen.print("Calibrating      ");
          tracking1.setPosition(0,degrees);
          tracking2.setPosition(0,degrees);
          wait (1, sec);
          Brain.Screen.setCursor(1,1);
          Brain.Screen.print("3               ");
          wait (1, sec);
          Brain.Screen.setCursor(1,1);
          Brain.Screen.print("2               ");
          wait (1, sec);
          Brain.Screen.setCursor(1,1);
          Brain.Screen.print("1               ");
          wait (1, sec);
          Brain.Screen.setCursor(1,1);
          Brain.Screen.print("Calibrated      ");
        }
      }
    } 
    //Skills
    if (Brain.Screen.xPosition()> 180){
      if (Brain.Screen.xPosition()< 300){
       if (Brain.Screen.yPosition()< 120){
         auton = 1;
         Brain.Screen.setCursor(1,1);
         Brain.Screen.setFillColor(white);
         Brain.Screen.setPenColor(black);
         Brain.Screen.print("Skills         ");
       }
      }
     }
     //Right side menu  
     if (Brain.Screen.xPosition()> 300){
      Menu2();
     }  
    }
    wait(20, msec);
  }
}
void IDK(void) {
  wait(1,sec);
}
void pre_auton(void) {
  vexcodeInit();
  intake.setVelocity(100,pct);
  initial_menu();
}
void autonomous(void) {
  Optical1.setLightPower(100, percent);

  if (auton == 1) {
    skills_auton();
  }
  else if (auton == 2) {
    left_blue();
  }
  else if (auton == 3) {
    left_red();
  }
  else if (auton == 4) {
    left_blue_elims();
  }
  else if (auton == 5) {
    left_red_elims();
  }
  else if (auton == 6) {
    right_blue();
  }
  else if (auton == 7) {
    right_red();
  }
  else if (auton == 8) {
    right_blue_elims();
  }
  else if (auton == 9) {
    right_red_elims();
  }
  else {
    //skills_auton();
    //left_red();
    //right_red_elims();
    //left_blue_elims();
    //right_blue();
    right_red();
    //left_blue();

    //optical sensor test
    
    /*while (1)
    {
      intake.spin(forward);
      Optical1.objectDetected(blue_detected);
      wait (3,sec);
    }*/
    

    //reset claw rotation
    //claw_mech.setVelocity(100,pct);
  }
  /*vex::task DrivetrainPID(PID_Drive);

  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);

  ResetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;*/
} 

int current_velocity = 100;
bool notstuck = true;
void usercontrol(void) {
  //enablePID_Drive = false;
  // User control code here, inside the loop
  while (1) {
    claw_mech.setVelocity(100, pct);
    intake.setVelocity(100, pct);
    Drivetrain.setDriveVelocity(100, pct);
    if (Controller1.ButtonUp.pressing()) {
      notstuck = true;
      if (notstuck == true) {
      claw_mech.setVelocity(100, pct);
      while (claw_tracking.position(degrees) >= 0){
            claw_mech.spin(reverse);
          }
          wait (.1,sec);
          claw_mech.spinFor(140,degrees);
          claw_mech.stop(hold);
      }
    }
    if (Controller1.ButtonRight.pressing()) {
        notstuck = false;
        wait (.1,sec);
        claw_mech.spinFor(60,degrees);
        claw_mech.stop(hold);
    }
    // Clamp toggle
    if (Controller1.ButtonY.pressing()) {
      if (Clamping == false){
        Clamping = true;
        wait(.1, sec);        
      }
       else if (Clamping == true){
        Clamping = false;
        wait(.1, sec);
      }
    }
    Clamp.set(Clamping);

    // Descore mech toggle    
  if (Controller1.ButtonB.pressing()) {
      if (elevate == false){
        elevate = true;
        wait(.1, sec);
      }
       else if (elevate == true){
        elevate = false;
        wait(.1, sec);
      }
    }
    descore_mech.set(elevate);

    // Elevation toggle
  if (Controller1.ButtonDown.pressing()) {
      if (tier_1 == false){
        while (claw_tracking.position(degrees) >= 0){
            claw_mech.spin(reverse);
          }
          wait (.1,sec);
          claw_mech.spinFor(240,degrees);
          claw_mech.stop(hold);
          wait(1, sec);
          elevation.set(true);
          wait(1, sec);
          claw_mech.spinFor(-140,degrees);
          claw_mech.stop(hold);
          tier_1 = true;
      }
       else if (tier_1 == true){
        tier_1 = false;
        wait(.1, sec);
      }
    }
    elevation.set(tier_1);
    // Intake Spinner
    if (Controller1.ButtonL1.pressing()) {
        intake.spin(forward);
        // insert function here
    }else if (Controller1.ButtonL2.pressing()) {
      intake.spin(reverse);
    }else if (true){
      intake.stop();
    }
  LeftDriveSmart.setVelocity(100, pct);
  RightDriveSmart.setVelocity(100,pct);
  
  
  
    wait(.1, sec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
   }
}



//kasen did not miss a semicolon - mckay

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