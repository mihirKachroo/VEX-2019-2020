#ifndef _MOTORSLEW_H_
#define _MOTORSLEW_H_

//include relevant source files
#include "motorslew.h"
#include "main.h"      

//request motor speed for the slewing task to achieve
void motorReq(int channel, int speed);

void motorslewing(void * parameter);  //manages speed of motors

#endif
