//include relevant source file
#include "motorslew.h"

//rate in which the motor steps
#define MOTOR_SLEW_RATE  10
//delay to request motor speed
#define MOTOR_TASK_DELAY 20

//requested motor speeds for all motors
int motor_req[11];
//current motor speed for all motors
int motor_speed[11];
//is this motor being slewed?
bool motor_slew[11];

void motorslewing(void * parameter) {              //motor slew function
  for(int i = 1; i <= 10; i++) {                   //resets all variables
    //resets all requested motor speeds
    motor_req[i] = 0;
    //resets all motor slewing states
    motor_slew[i] = false;
    //rests all motors current speed
    motor_speed[i] = 0;
  }
  //repeating function through the program
  while(true){
    //check through every motor in the program
    for(int i = 1; i <= 10; i++){
      if(motor_slew[i]) {                          //does it require slewing
        //is the requested speed greater than the current speed?
        if(motor_req[i] > motor_speed[i]) {
          motor_speed[i] += MOTOR_SLEW_RATE;       //add the slew value
          //has the slew rate over-corrected
          if(motor_speed[i] > motor_req[i]) {
            //then set the requested speed
            motor_speed[i] = motor_req[i];
            //the motor need not be slewed anymore, so request can be turned
            //  off
            motor_slew[i] = false;
          }
        } else if(motor_req[i] < motor_speed[i]) { //is a slower speed required
          motor_speed[i] -= MOTOR_SLEW_RATE;       //lower the speed reasonably
          if(motor_speed[i] < motor_req[i]) {      //has it over-corrected
            //set the actual requested speed
            motor_speed[i] = motor_req[i];
            //the motor need not be slewed anymore, so request can be turned
            //  off
            motor_slew[i] = false;
          }
        } else {
          //set the required speed as last slewing already set the required speed
          motor_speed[i] = motor_req[i];
          //required speed set, no longer needs to be slewed
          motor_slew[i] = false;
        }
      }
      motorSet(i, motor_speed[i]);                 //finally set the motorspeed
    }
    delay(MOTOR_TASK_DELAY);
  }
}

void motorReq(int channel, int speed) {            //speed request
    //set the requested speed for the requested motor
    speed = speed > 127 ? 127 : speed;
    speed = speed < -127 ? -127 : speed;
    motor_req[channel] = speed;
    motor_slew[channel] = true;                    //motor requires slewing
}
