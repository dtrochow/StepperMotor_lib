#include "inc/StepperMotor.h"

/**
 * Make one step in chosen direction.
 * It is made by setting low state on CLK driver pin. 
 * @TODO Describe how to set STEP on stepper motor driver (include description to differ drivers)
 */
void make_step(bool dir, StepperMotor_t motor)
{
    gpio_put(motor.dirPin, dir);
    gpio_put(motor.stepPin, 0);
    sleep_us(1);
    gpio_put(motor.stepPin, 1);
}

/**
 * Motor initialization.
 * @TODO Describe what this inicializzation doing.
 */
void init_motor(StepperMotor_t *motor, uint stepPin_, uint dirPin_, uint enPin_, float current_, unsigned int microstep_, unsigned int decaySetting_)
{
    motor->stepPin = stepPin_;
    motor->dirPin = dirPin_;
    motor->enPin = enPin_;
    motor->current = current_;
    motor->microstep = microstep_;
    motor->decaySetting = decaySetting_;

    gpio_init(motor->stepPin);
    gpio_init(motor->dirPin);
    gpio_init(motor->enPin);

    gpio_set_dir(motor->stepPin, GPIO_OUT);
    gpio_set_dir(motor->dirPin, GPIO_OUT);
    gpio_set_dir(motor->enPin, GPIO_OUT);

    gpio_put(motor->stepPin, 0);
    gpio_put(motor->dirPin, 0);
    gpio_put(motor->enPin, 0);
}