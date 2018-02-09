/*
 * PID.cpp
 *
 * Created: 1/29/2016 1:04:42 PM
 *  Author: YinYang
 */  
#include "PID.h"
#include "headers.h"

void constrain(float &val,float minVal,float maxVal)
{
	if(val < minVal)
		val = minVal;
	if(val > maxVal)
		val = maxVal;
}

void PID::Initialize()
{
	kp         =  0;
	ki         =  0;
	kd         =  0;
	error      =  0;
	errSum     =  0;
	lastinput  =  0;
	offset     =  0;
	setPoint   =  0;
	PID_Flag   =  true;
}
void PID::Set_PID(float KP,float KI,float KD)
{
	kp = KP;
	ki = KI;
	kd = KD;
}
float PID::Compute_PID(float input)
{
		error   = setPoint - input;
		/*errSum += error;*/
		Iterm += (ki*error);
		constrain(errSum,-25,25);
	
		float dErr = (input - lastinput);
	
		//Compute PID Output
		//float output;
		//if (fabs(error)>3)
		//{
			output = kp * error + Iterm - kd * dErr + offset;
		//}
		//else
		//output=0;
	
		constrain(output,minOut,maxOut);
		//Remember some variables for next time
		lastinput = input;
	return output;
}
