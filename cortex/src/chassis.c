#include "chassis.h"                   //include relevant source file

void chassisSet(int left, int right) { //sets the speed of the drive system
  //motordrive systems
  //Note: Gotta reverse left 1 and right 3 on the bot
  left*=-1;
  right*=0.8;
  //right*=-1;
  float speedChange=1;
  motorReq(leftMotor1, left*speedChange);
  motorReq(rightMotor1, -right);
  motorReq(leftMotor2, left*speedChange);
  motorReq(rightMotor2, -right);
  motorReq(leftMotor3, left*speedChange);
  motorReq(rightMotor3, -right);
  motorReq(leftMotor4, left*speedChange);
  motorReq(rightMotor4, -right);
}
