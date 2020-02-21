#ifndef _CHASSIS_H_
#define _CHASSIS_H_

//include relevant source files
#include "main.h"
#include "motorslew.h"

#define leftMotor1  2
#define rightMotor1 5
#define leftMotor2  2
#define rightMotor2 5
#define leftMotor3  3
#define rightMotor3 6
#define leftMotor4  3
#define rightMotor4 6
#define leftIntake 8
#define rightIntake 9
#define liftMechanism 7


//set speed of dlrive system
void chassisSet(int left, int right); //sets the speed of the drive system
  //motordrive systems



#endif
