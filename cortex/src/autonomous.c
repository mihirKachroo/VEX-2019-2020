#include "autonomous.h"

//Note: Add route for whether the robot turns left or right
void myAuton(int route, int position) {
  printf("runnn forrest runnn");
  //Gotta keep right speed double left speed becuase of chassis parameters.


  //Moves forward while intaking
  moveDrive(50,50);
  motorReq(rightIntake, 30);
  motorReq(leftIntake, -30);
  delay ( 4500 ) ;

  //ADD ROUTE FOR IT TO TURN LEFT
  //Turns right
  moveDrive(0,50);
  delay(2000);
  moveDrive(0,0);

  //Moves towards the goal
  moveDrive(50,50);
  delay(3000);

  //Launches it out
  moveDrive(0,0);
  motorReq(rightIntake, 0);
  motorReq(leftIntake, 0);
  motorReq(liftMechanism, 50);
  delay(1500);

  //Stops
  motorReq(liftMechanism, 0);
  delay ( 500 ) ;

  //moves back
  moveDrive(-50,-50);

  delay(2000);

  //Stops
  moveDrive(0,0);

}
int moveDrive(int rSpeed, int lSpeed){
  lSpeed*=-0.7;

  motorReq(leftMotor1, lSpeed);
  motorReq(rightMotor1, -rSpeed);
  motorReq(leftMotor2, lSpeed);
  motorReq(rightMotor2, -rSpeed);
  motorReq(leftMotor3, lSpeed);
  motorReq(rightMotor3, -rSpeed);
  motorReq(leftMotor4, lSpeed);
  motorReq(rightMotor4, -rSpeed);
}
