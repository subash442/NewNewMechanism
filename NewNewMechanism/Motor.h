#ifndef MOTOR_H
#define MOTOR_H


	#include "headers.h"

     #define ICR_TOP   249
     #define MAX_VALUE 249
     #define MIN_VALUE 0


	//For motor 1


    #define DD_F1  B,4                           //B,4
    #define DD_B1  B,7                           //B,7

    #define DD_PWM1			B,5
    #define PWM_TCCRA1		TCCR1A
    #define PWM_TCCRB1		TCCR1B
    #define PWM_ICR1		ICR1
    #define PWM_OCR1		OCR1A

    #define PWM_1COM0		COM1A0
    #define PWM_1COM1		COM1A1

    #define PWM_1WGM0		WGM10
    #define PWM_1WGM1		WGM11
    #define PWM_1WGM2		WGM12
    #define PWM_1WGM3		WGM13
    #define PWM_1CS0		CS10
    #define PWM_1CS1		CS11
    #define PWM_1CS2		CS12

    


class Motor
{

     private:
		
     public:
        void Initialise();

        void InitPWM();

        void SetForwardDirection();
        void SetReverseDirection();
        void StopMotor();
        void SetOcrValue(int x);
};

#endif // MOTOR_H
