#include "encoder_pid.h"        //include relevant header
#include "tracking.h"

//sets the PID controller values for each instance
void pidSet(pid_info* pid,
  double p, double i, double d,
  double motor) {
    pid->p = p;                 //sets the proportional value
    pid->i = i;                 //sets the integral value
    pid->d = d;                 //sets the derivative value
    pid->motor = motor;         //stores which motor this PID info is relevant to
  }

int encoderLeftOffset;
int encoderRightOffset;

void encodeMe(int distLeft, int distRight) {
  int tempLeftDist;
  int tempRightDist;
  while ((distLeft > 0) || (distRight > 0)) { 
    if (distLeft>500) {
      tempLeftDist = 500;
      distLeft -= 500;
    } else { 
      tempLeftDist = distLeft;
      distLeft = 0;
    }
    if (distRight > 500) {
      tempRightDist = 500;
      distRight -= 500;
    } else {
      tempRightDist = distRight;
      distRight = 0;
    }
    encoderMotorAutonomous(autonBackLeft, autonBackRight, tempLeftDist, tempRightDist);
  }
}

int getEncoderLeft() {
  return encoderGet(encoderLeft) + encoderLeftOffset;
}
int getEncoderRight() {
  return encoderGet(encoderRight) + encoderRightOffset;
}

/*
  This function uses a PID controller using information from the PID structure
  to move the robot based on sensor-readings (typically encocder-ticks)
*/
typedef struct pidData {
  float sense;
  int lastError, errorLongTimeAgo;
  int integral;
  int error, derivative, speed;
  int target, lastTarget, timePassed;
} pidData;

void Reset(struct pidData* data)
{
  data->derivative = 0;
  data->error = 0;
  data->integral = 0;
  data->lastError = 0;
  data->errorLongTimeAgo = 0;
  data->sense = 0;
  data->speed = 0;
  data->target = 0;
  data->lastTarget = 0;
  data->timePassed = 0;
}

struct pidData leftData;
struct pidData rightData;
struct pidData rightDataAuton;
struct pidData leftDataAuton;
pid_info pid;
pid_info pid_other;
int timeout;

bool CalculatePIDAuto(pidData* data, pid_info pid)
{
  data->timePassed += 2;
  //calculate the error from target to current readings
data->error = data->target - data->sense;
//printf("%d, %d, %f\n", data->error, data->target, pid.p);
//printf("\nfinding the data error: %d, %f", data->error, data->sense);

data->integral += data->error;                  //add the error to the integral
//find the derivative by calculating the difference from the previous two
//  errors
data->derivative = data->error - data->lastError;

//disable the integral until it comes into a usable range
if(data->error == 0 || (abs(data->error) > (127/2))) { data->integral = 0; }

//put the whole PID shenanigan together and calculate the speed
data->speed = (pid.p*data->error) + (pid.i*data->integral) + (pid.d*data->derivative);

//if the previous two errors were 0, then the robot has probably stopped,
//  so exit the program
if (((data->error == data->errorLongTimeAgo))) { data->speed = 0; return false; }

//end of loop, current error becomes the last error for the next run
if(data->timePassed%500 < 2)
{
  data->errorLongTimeAgo = data->error;
  /*printf("Updated error long time ago: %d: \n", data->errorLongTimeAgo);*/
}
data->lastError = data->error;
return true;
}

pidData CalculatePID(pidData data, pid_info pid)
{
    /*if (millis()%40 <= 2)
      printf("PID prop: %f\n", pid.p);*/
    //calculate the error from target to current readings
    data.error = data.target - data.sense;
    data.integral += data.error;                  //add the error to the integral
    //find the derivative by calculating the difference from the previous two
    //  errors
    data.derivative = data.error - data.lastError;

    //disable the integral until it comes into a usable range
    if(data.error == 0 || (abs(data.error) > (127/2))) { data.integral = 0; }

    //put the whole PID shenanigan together and calculate the speed
    data.speed = (pid.p*data.error) + (pid.i*data.integral) + (pid.d*data.derivative);

    //if the previous two errors were 0, then the robot has probably stopped,
    //  so exit the program
    if ((abs(data.error) <= 0 && abs(data.lastError) <= 0) || (data.target == data.lastTarget && data.error == data.lastError)) {
      data.speed = 0;
      data.target = data.sense;
      //printf("exit speed\n");
    }

    //end of loop, current error becomes the last error for the next run
    data.lastError = data.error;
    if(millis()%400 <= 3)
    { data.lastTarget = data.target;
    //  printf("Right: E %d, N %f, T %d, S %d\n", rightData.error, rightData.sense, rightData.target, rightData.speed);
    //printf("Left: E %d, N %f, T %d, S %d\n", leftData.error, leftData.sense, leftData.target, leftData.speed);
    }

    return data;
}


void changeOffsets(int right, int left)
{
  encoderRightOffset += right;
  encoderLeftOffset += left;
}

void changeRightTarget(int target){
  rightData.target += target;
}

void changeLeftTarget(int target){
  leftData.target += target;
}

void encoderMotorAutonomous(pid_info leftPID, pid_info rightPID, int targetLeft, int targetRight) {
  delay(20);
  encoderReset(encoderRight);
  encoderReset(encoderLeft);

  //function();

  //timeout = 10*((abs(targetLeft) + abs(targetRight)) / 2)/2.54 + millis();
  //printf("returned timeout%d", timeout);

  Reset(&rightDataAuton);
  Reset(&leftDataAuton);
  rightDataAuton.target = targetRight;
  leftDataAuton.target = targetLeft;

  //variable holding sensor information (encoder)
  //resets the integral value
  bool runRight = true;
  bool runLeft = true;//start the PID controller
  //initialize the error, derivative and resulting speed values

  while(runRight || runLeft) {
    //printf("PID data: %f, target: %d", pid->p, target);
    rightDataAuton.sense = encoderGet(encoderRight);
    leftDataAuton.sense = encoderGet(encoderLeft); //get encoder readings
    //printf("\nsense%f, %f", *(&rightData.sense), *(&leftData.sense));

    if(runRight) {runRight = CalculatePIDAuto(&rightDataAuton, rightPID);}
    if(runLeft) {runLeft = CalculatePIDAuto(&leftDataAuton, leftPID);}
    if (millis()%25 <= 2) {
      printf("\nRight: %d,%d\n", rightDataAuton.speed, rightDataAuton.target);
      printf("Left: %d,%d, %f \n", leftDataAuton.speed, leftDataAuton.target, leftPID.p);
    }

    chassisSet(leftDataAuton.speed,rightDataAuton.speed);        //request the calculated motor speed
    //tracking();
    delay(2);
  }
}


void encoderMotor(void * parameter) {
  encoderReset(encoderRight);
  encoderReset(encoderLeft);

  Reset(&rightData);
  Reset(&leftData);

  /*rightData.turnMultiplier = forwardRight == true ? 1 : -1;
  leftData.turnMultiplier = forwardLeft == true ? 1 : -1;
  rightData.target = rightTarget;
  leftData.target = leftTarget;*/

  //variable holding sensor information (encoder)
  //resets the integral value
  //initialize the error, derivative and resulting speed values
  pid = driveStraightRight;
  pid_other = driveStraightLeft;

  while(1) {
    rightData.sense = getEncoderRight();
    leftData.sense = getEncoderLeft(); //get encoder readings
    rightData = CalculatePID(rightData, pid);
    leftData = CalculatePID(leftData, pid_other);
    if (millis()%20 <= 3) {
      //printf("Right: E %d, N %d, S %d\n", rightData.error, rightData.sense, rightData.speed);
      //printf("Left: E %d, N %d, S %d\n", leftData.error, leftData.sense, leftData.speed);
    }

    chassisSet(leftData.speed, rightData.speed);        //request the calculated motor speed
    tracking();
    delay(2);
  }
}
//calculates the ratio in which the robot moves in proportion to the number of
//  ticks
void intRatio(int encoderTicks, int angle) {
  ratio = encoderTicks / angle;          //simple ration calculation
}

//use encoders to try to make an accurate turn
void encoderTurn(float angle) {
  encoderMotorAutonomous(autonStraightLeft, autonStraightRight, angle*encoder_turn_constant, -angle*encoder_turn_constant);
  //negate the angle as the motor will turn the opposite way
  //encoderMotor(motor2, (angle*ratio));
}
