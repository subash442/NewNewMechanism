/*
 * NewNewMechanism.cpp
 *
 * Created: 1/22/2018 5:15:53 AM
 * Author : Deepak Chand
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "uart.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"


Motor   M;
Encoder E;
PID    Speed_PID;
PID    Angle_PID;


void Initialize_TCRT_Interrupt(void);
void GoToHome(void);
void Throw(void);
void RampDown(void);

#define TCRT       D,1
#define Pneumatic  L,0

volatile bool PIDFlag   = true;
volatile bool ThrowFlag = false;

unsigned char data;
int RevolutionCount;
int Speed;


int main(void)
{
	M.Initialise();
	
	E.Encoder_Initialize();
	
	Initialize_TCRT_Interrupt();
	OUTPUT(Pneumatic);
	SET(Pneumatic);
	
	Speed_PID.Initialize();
	Speed = 30;
	Speed_PID.Set_Range(-249,249);
	Speed_PID.Set_PID(8.05,0.115,4.299);                               //7.64,0.005,2.664   // 10.55,0.059,0.135  15.30,0.074,0.619  14.30,0.149,1.264
	
	Angle_PID.Initialize();
	Angle_PID.Set_Range(-249,249);
	Angle_PID.Set_PID(2.09,0,0.09);
	
	sei();
	uart0_init(UART_BAUD_SELECT(9600,F_CPU));
//	uart3_init(UART_BAUD_SELECT(9600,F_CPU));
	uart0_puts("Give Command!!!\n\r");
   
   GoToHome();
   
    while (1) 
    {
		data = uart0_getc();
		switch (data)
		{
			case 'a':
				{
					PIDFlag = false;
					Angle_PID.SetSetPoint(45);
					E.Angle_count = 0;
					break;
				}
			case 'g':
			{
				PIDFlag = true;
				Speed_PID.SetSetPoint(Speed);
				break;
			}
			case 's':
			{
				PIDFlag = true;
				Speed_PID.SetSetPoint(0);
				M.StopMotor();
				RevolutionCount = 0;
				break;
			}
			case 'h':
			{
				GoToHome();
				break;
			}
			
			case 'q':
			{
				CLEAR(Pneumatic);
				break;
			}
			case 'w':
			{
				SET(Pneumatic);
				break;
			}
			
				default:break;
		}
		data = 0;
// 		
// 		uart0_putint(E.ExtraCount);
// 		uart0_putc(' ');
		
		if (RevolutionCount == 3 && abs(E.Angle_count) >= 270)                         //775 at opposite direction of motor     ThrowingMeter >= 3330     RevolutionCount == 3 && abs(E.Angle_count) >= 270
		{
			Throw();
		}
		
		if (RevolutionCount == 4 && ThrowFlag == true)
		{
			
			do
			{
				Speed -= 5;
				Speed_PID.SetSetPoint(Speed);
				
				if (Speed < 0)
				{
					ThrowFlag = false;
					Speed_PID.SetSetPoint(0);
				}
				
				if (Speed_PID.PID_Flag == true && PIDFlag == true)
				{
					M.SetOcrValue(Speed_PID.Compute_PID(E.Encoder_get_speed()));
					Speed_PID.PID_Flag = false;
				}
				
			} while (ThrowFlag);
		}
		
		if (Speed_PID.PID_Flag == true && PIDFlag == true)
		{
			M.SetOcrValue(Speed_PID.Compute_PID(E.Encoder_get_speed()));
			Speed_PID.PID_Flag = false;
		}
		if (Angle_PID.PID_Flag == true && PIDFlag == false)
		{
			M.SetOcrValue(Angle_PID.Compute_PID(E.Encoder_Get_angle()));
			Angle_PID.PID_Flag = false;
		}
		
    }
}

ISR(TIMER0_COMPA_vect)
{
	Speed_PID.PID_Flag = true;
	Angle_PID.PID_Flag = true;
	E.Encoder_update_Speed();
}

ISR(ENCODER_INTERRUPT_VECT)
{
	E.Encoder_Increase_Pulse_Counter();
	E.Encoder_Increase_Angle_Counter();
}


void Initialize_TCRT_Interrupt()
{
	INPUT(TCRT);								//Interrupt Pin as Input
	SET(TCRT);									//Pull_UP
	
	EICRA |= (1<<ISC11);						//Falling Edge Interrupt
	EIMSK |= (1<<INT1);
	EIFR  |= (1<<INTF1);
}

ISR(INT1_vect)
{
	E.Angle_count = 0;
	RevolutionCount++;
}

void GoToHome()
{
	if (E.Encoder_Get_angle() <= 0)
	{
		while(READ(TCRT))
		{
			/*	M1.SetReverseDirection();*/
			M.SetOcrValue(30);
			//E.Angle_count = 0;
			
		}
		M.SetOcrValue(0);
		E.Angle_count = 0;
		RevolutionCount = 0;
		E.ExtraCount = 0;
	}
	else if (E.Encoder_Get_angle() > 0)
	{
		while(READ(TCRT))
		{
			/*M1.SetForwardDirection();*/
			M.SetOcrValue(-30);
			//E.Angle_count = 0;
		}
		M.SetOcrValue(0);
		E.Angle_count = 0;
		RevolutionCount = 0;
		E.ExtraCount = 0;
	}
	RevolutionCount = 0;
	Speed_PID.SetSetPoint(0);
	M.StopMotor();
}

void Throw()
{
	CLEAR(Pneumatic);
	ThrowFlag = true;
}

void RampDown()
{	
	
}