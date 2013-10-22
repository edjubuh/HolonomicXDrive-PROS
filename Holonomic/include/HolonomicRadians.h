/*
* HolonomicRadians.h
*
*  Created on: Oct 13, 2013
*      Author: Elliot
*/

#ifndef HOLONOMICRADIANS_H_
#define HOLONOMICRADIANS_H_

#define maxMotorSpeed 	127
#define numberOfMotors 	4

#define	PI	3.14159265358979

#include <math.h>
#include "CortexDefinitions.h"

//**--------------------- SUPPORT FUNCTIONS ----------------------**//
double FindMaxdouble(double a [])
{
	double maxdouble = 0;
	for (int i = 0; i < sizeof(a); i++)
	{
		if (a[i] > maxdouble) maxdouble = a[i];
	}
	return maxdouble;
}
//**------------------- MAIN FUNCTIONS --------------------------- **//

/*
* Converts radians, speed, and rotation to an x-drive holonomic configuration
* to enable robot to go in any direction at speed
*  
* Requires CortexDefinition.h to be updated (uses FRONT_LEFT_MOTOR, etc. definitions)
*  
* @param radians The heading of motion, where 0 is forward, PI/2 is right (see compass)
* @param speed The speed of motion, from 0 to 1
* @param rotation The speed of rotation of the bot, from -127 to 127
*/
void RadianOutput(double radians, double speed, int rotation)
{
	if (speed > 0)
	{
		double frontLeftOutput = ( (-1 * maxMotorSpeed) * cos((PI / 4) - radians) ), /*added "-1*" for clarification*/
		       frontRightOutput = (maxMotorSpeed * cos((PI / 4) + radians) ),
		       rearRightOutput = (maxMotorSpeed * cos((PI / 4) - radians) ),
		       rearLeftOutput = ( (-1 * maxMotorSpeed) * cos( (PI / 4) + radians) );

		frontLeftOutput += rotation;
		frontRightOutput += rotation;
		rearRightOutput += rotation;
		rearLeftOutput += rotation;

		double output[4] = { frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput };
		double maxValue = FindMaxdouble(output);
		speed *= maxMotorSpeed / maxValue;

		frontLeftOutput *= speed;
		frontRightOutput *= speed;
		rearLeftOutput *= speed;
		rearRightOutput *= speed;
		motorSet(FRONT_LEFT_MOTOR, frontLeftOutput);
		motorSet(FRONT_RIGHT_MOTOR, frontRightOutput);
		motorSet(REAR_LEFT_MOTOR, rearLeftOutput);
		motorSet(REAR_RIGHT_MOTOR, rearRightOutput);
	}
	else if (rotation > DEADBAND || rotation < -DEADBAND)
	{
		motorSet(FRONT_LEFT_MOTOR, rotation);
		motorSet(FRONT_RIGHT_MOTOR, rotation);
		motorSet(REAR_LEFT_MOTOR, rotation);
		motorSet(REAR_RIGHT_MOTOR, rotation);
	}
	else
	{
		motorStop(FRONT_LEFT_MOTOR);
		motorStop(FRONT_RIGHT_MOTOR);
		motorStop(REAR_LEFT_MOTOR);
		motorStop(REAR_RIGHT_MOTOR);
	}
}

double getJoyPolarRadians() 
{ 
	return atan2((double) joystickGetAnalog(1, 2), (double) joystickGetAnalog(1, 1)); 
}
double getJoyPolarSpeed() 
{ 
	return (maxMotorSpeed / sqrt((double) (joystickGetAnalog(1, 2) ^ 2) + (double) (joystickGetAnalog(1, 1) ^ 2))); 
}

#endif HOLONOMICRADIANS_H_
