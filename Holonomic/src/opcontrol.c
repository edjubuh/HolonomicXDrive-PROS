/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2013, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "HolonomicRadians.h"
#include "CortexDefinitions.h"

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
 * the schedular is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	bool lcdLeftPressed = false;
	bool lcdCenterPressed = false;
	bool lcdRightPressed = false;

	bool joy8LPressed = false;
	bool joy8UPressed = false;
	bool joy8RPressed = false;

	bool killSwitch = false;
	int radianMultiplier = 0;

	lcdSetText(uart1, 1, "Cycle PI/4");
	while (true) {
		if (lcdReadButtons(uart1) == LCD_BTN_LEFT && !lcdLeftPressed)
		{
			radianMultiplier--;
			lcdLeftPressed = true;
		}
		else if (lcdReadButtons(uart1) != LCD_BTN_LEFT) lcdLeftPressed = false;

		if (lcdReadButtons(uart1) == LCD_BTN_CENTER && !lcdCenterPressed)
		{
			killSwitch = true;
			lcdCenterPressed = true;
		}
		else if (lcdReadButtons(uart1) != LCD_BTN_CENTER) lcdCenterPressed = false;

		if (lcdReadButtons(uart1) == LCD_BTN_RIGHT && !lcdRightPressed)
		{
			radianMultiplier++;
			lcdRightPressed = true;
		}
		else if (lcdReadButtons(uart1) != LCD_BTN_RIGHT) lcdRightPressed = false;

		if (joystickGetDigital(1, 8, JOY_LEFT) && !joy8LPressed)
		{
			radianMultiplier--;
			joy8LPressed = true;
		}
		else if (!joystickGetDigital(1, 8, JOY_LEFT)) joy8LPressed = false;

		if (joystickGetDigital(1, 8, JOY_UP) && !joy8UPressed)
		{
			killSwitch = true;
			joy8UPressed = true;
		}
		else if (!joystickGetDigital(1, 8, JOY_UP)) joy8UPressed = false;

		if (joystickGetDigital(1, 8, JOY_RIGHT) && !joy8RPressed)
		{
			radianMultiplier++;
			joy8RPressed = true;
		}
		else if (joystickGetDigital(1, 8, JOY_RIGHT)) joy8RPressed = false;

		if (radianMultiplier > 7) radianMultiplier = 0;
		if (radianMultiplier < 0) radianMultiplier = 7;

		switch (radianMultiplier)
		{
		case 0: lcdSetText(uart1, 2, "<      0       >"); break;
		case 1: lcdSetText(uart1, 2, "<     PI/4     >"); break;
		case 2: lcdSetText(uart1, 2, "<     PI/2     >"); break;
		case 3: lcdSetText(uart1, 2, "<     3PI/4    >"); break;
		case 4: lcdSetText(uart1, 2, "<      PI      >"); break;
		case 5: lcdSetText(uart1, 2, "<     5PI/4    >"); break;
		case 6: lcdSetText(uart1, 2, "<     3PI/2    >"); break;
		case 7: lcdSetText(uart1, 2, "<     7PI/4    >"); break;
		default:
			lcdSetText(uart1, 2, "     ERROR      "); 
			motorStopAll(); delay(100); 
			killSwitch = true; 
			radianMultiplier = 0;
			break;
		}

		if (killSwitch) lcdSetText(uart1, 2, "KILLED");
		if (!killSwitch) RadianOutput(((float) radianMultiplier*(PI / 4)), 1, 0);
		else  motorStopAll();
		delay(20);
	}
}