/***************************************
 * main.c
 * rev 1.0 Dominik 2021
 *  ************************************/

#include <stdio.h>
#include "pico/stdlib.h"
#include "inc/StepperMotor.h"
#include "pico/multicore.h"
#include "pico/time.h"


const uint LED_PIN = 25;

void core1_entry()
{
    while(1)
    {
        ControlLoop();
    }
}

int main(void)
{
    StepperMotor_t motor1;
    
    InitMotor(&motor1, 11, 12, 13, 1.5, 12, 200, 2, 50, 0.2, 20);
    SetMotorSpeedDeg(&motor1, 90);

    // multicore_launch_core1(core1_entry);

    int revolution = 0;

    //SetMotorSpeedDeg(&motor1, 10);
    multicore_launch_core1(core1_entry);

    while(true)
    {
        // RotateMotor_angleDeg(&motor1, 360, CLOCKWISE);
        // revolution ++;
        // printf("Revolution %d ;-)\n", revolution);
        // sleep_ms(1000);
    }
}
 