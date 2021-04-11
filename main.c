/***************************************
 * main.c
 * rev 1.0 Dominik 2021
 *  ************************************/

#include "pico/stdlib.h"
#include "inc/StepperMotor.h"

int main(void)
{
    StepperMotor_t motor1;
    
    init_motor(&motor1, 11, 12, 13, 1.5, 8, 50);

    const uint LED_PIN = 25;

    //Inicjalizacja we/wy mikrokontrolera
    gpio_init(LED_PIN);
    //Ustawienie pinów uc jako wyjścia
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while(true)
    {
        make_step(1, motor1);
        sleep_us(50);
    }
}
