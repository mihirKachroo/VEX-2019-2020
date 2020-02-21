/** @file opcontrol.* @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *d
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "encoder_pid.h"
#include "chassis.h"
#include "tracking.h"
#include "autonomous.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */

float encoderConstant = 0.015;

void operatorControl() {
	autonomous();

	encoderReset(encoderRight);
	encoderReset(encoderLeft);
	taskCreate(motorslewing, TASK_DEFAULT_STACK_SIZE, NULL,	TASK_PRIORITY_HIGHEST);

	int joythresh = 25;
	bool run = true;
	int intakeSpeed = 125;
	int reverseMultiplier = 1;

	while(true) {

		printf("Running... mom");

		run = true;
		if (joystickGetDigital(1, 7, JOY_LEFT))
		{
			run = true;
		}
		if (run) {
			float rightSpeed = 0;
			float leftSpeed = 0;

if (abs(joystickGetAnalog(1, 3)) > joythresh){
				leftSpeed = joystickGetAnalog(1,3);
			} else {
				leftSpeed = 0;
			}
			if (abs(joystickGetAnalog(1, 2)) > joythresh){
				if(abs(joystickGetAnalog(1, 2)) > 108)
				{
					rightSpeed = joystickGetAnalog(1, 2);
				}
				else
				{
					rightSpeed = joystickGetAnalog(1, 2);
				}
			} else {
				rightSpeed = 0;
			}
			if (joystickGetDigital(1, 5, JOY_UP) || joystickGetDigital(1, 6, JOY_UP))
			{
				leftSpeed /= 3;
				rightSpeed /=3;
			}

			//Press to choose which side it goes from -- useless
			if (joystickGetDigital(1, 8, JOY_LEFT)) {
				reverseMultiplier = -1;
			}
			if (joystickGetDigital(1, 8, JOY_RIGHT)) {
				reverseMultiplier = 1;
			}

			//chassisSet(leftSpeed, rightSpeed);
				if(reverseMultiplier == 1)
				{
					chassisSet(leftSpeed * 0.5, rightSpeed);
				}
				else
				{
					chassisSet(rightSpeed * 0.5, leftSpeed);
				}
			delay(2);

			//Intake mechanism
			if (joystickGetDigital(1, 5, JOY_DOWN)) {
				motorReq(rightIntake, intakeSpeed);
				motorReq(leftIntake, -intakeSpeed);
			}
			else if(joystickGetDigital(1, 6, JOY_DOWN)) {
				motorReq(rightIntake, -intakeSpeed);
				motorReq(leftIntake, intakeSpeed*3);
			}
			else {
				motorReq(rightIntake, 0);
				motorReq(leftIntake, 0);
			}

		 //The lifting mechanism
			if (joystickGetDigital(1, 5, JOY_UP)) {
				motorReq(liftMechanism, intakeSpeed/1.5);
			}
			else if(joystickGetDigital(1, 6, JOY_UP)) {
				motorReq(liftMechanism, -intakeSpeed/1.5);
			}
			else {
				motorReq(liftMechanism, 0);
			}

			if (joystickGetDigital(1, 7, JOY_RIGHT))	{
				run = false;
				rightSpeed = 0;
				leftSpeed = 0;
			}

		}
	}
}
