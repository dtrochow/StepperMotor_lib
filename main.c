/***************************************
 * main.c
 * rev 1.0 Dominik 2021
 *  ************************************/

#include <stdio.h>
#include "pico/stdlib.h"
#include "inc/StepperMotor.h"
#include "pico/multicore.h"


const uint LED_PIN = 25;

bool repeating_timer_callback(struct repeating_timer *t)
{
    return true;
}

void core1_entry()
{
    stdio_init_all();
    bool x = 1;
    //Inicjalizacja we/wy mikrokontrolera
    gpio_init(LED_PIN);
    //Ustawienie pinów uc jako wyjścia
    gpio_set_dir(LED_PIN, GPIO_OUT);

    //struct repeating_timer timer;
    //add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);

    while(1)
    {
        sleep_ms(500);
        x^=1;
        gpio_put(LED_PIN, x);
    }
}

int main(void)
{
    StepperMotor_t motor1;
    
    InitMotor(&motor1, 11, 12, 13, 1.5, 12, 200, 2, 50);

    multicore_launch_core1(core1_entry);

    int revolution = 0;

    while(true)
    {
        RotateMotor_angleDeg(&motor1, 360, CLOCKWISE);
        revolution ++;
        printf("Revolution %d ;-)\n", revolution);
        sleep_ms(1000);
    }
}
