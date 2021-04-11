/***************************************
 * main.c
 * rev 1.0 Dominik 2021
 *  ************************************/

#include "pico/stdlib.h"
#include "inc/StepperMotor.h"

int main(void)
{
    StepperMotor_t motor1;
    
    InitMotor(&motor1, 11, 12, 13, 1.5, 12, 200, 8, 50);

    while(true)
    {
        RotateMotor_angleDeg(&motor1, 45, CLOCKWISE);
        sleep_ms(1000);
    }
}
