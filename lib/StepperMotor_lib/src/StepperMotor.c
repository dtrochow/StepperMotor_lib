#include "inc/StepperMotor.h"

/**
 * Make one step in chosen direction.
 * It is made by setting low state on CLK driver pin. 
 * @TODO Describe how to set STEP on stepper motor driver (include description to differ drivers)
 */
void MakeStep(bool dir, StepperMotor_t motor)
{
    gpio_put(motor.dirPin, dir);
    gpio_put(motor.stepPin, 0);
    sleep_us(3);
    gpio_put(motor.stepPin, 1);
}

/**
 * Motor initialization.
 * @TODO Describe what this inicializzation doing.
 */
void InitMotor(StepperMotor_t *motor, uint stepPin_, uint dirPin_, uint enPin_,
                double current_, double nVoltage_, unsigned int revSteps_, 
                unsigned int microstep_, unsigned int decaySetting_)
{
    motor->stepPin = stepPin_;
    motor->dirPin = dirPin_;
    motor->enPin = enPin_;
    motor->current = current_;
    motor->microstep = microstep_;
    motor->decaySetting = decaySetting_;
    motor->nVoltage = nVoltage_;
    motor->revSteps = revSteps_;
    motor->speed_rad = 0;   //Set initial motor speed to 0 rad/s
    motor->speed_m = 0;     //Set initial motor speed to 0 m/s

    //Initialize stepper motor controler outputs
    gpio_init(motor->stepPin);                  //Initialize gpio
    gpio_init(motor->dirPin);
    gpio_init(motor->enPin);
    gpio_set_dir(motor->stepPin, GPIO_OUT);     //Set gpio as outputs
    gpio_set_dir(motor->dirPin, GPIO_OUT);
    gpio_set_dir(motor->enPin, GPIO_OUT);
    gpio_put(motor->stepPin, 0);                //Set initial value to low
    gpio_put(motor->dirPin, 0);
    gpio_put(motor->enPin, 0);
}

/**
 * Rotete stepper motor by given angle in degrees.
 */
void RotateMotor_angleDeg(StepperMotor_t *motor, double angleDeg, bool dir)
{
    unsigned int steps = GetStepsFromAngleDeg(angleDeg, motor->revSteps, motor->microstep);
    for(int i = 0; i < steps; i++)
    {
        MakeStep(dir, *motor);
        sleep_us(500);
    }
}

/**
 * Rotate stepper motor by given angle in radians.
 */
void RotateMotor_angleRad(StepperMotor_t *motor, double angleRad, bool dir)
{
    unsigned int steps = GetStepsFromAngleRad(angleRad, motor->revSteps, motor->microstep);
    for(int i = 0; i < steps; i++)
    {
        MakeStep(dir, *motor);
        sleep_us(500);
    }
}

/**
 * Rotate stepper motor by given steps count.
 */
void RotateMotor_steps(StepperMotor_t *motor, double steps, bool dir)
{
    for(int i = 0; i < steps; i ++)
    {
        MakeStep(dir, *motor);
        sleep_us(500);
    }
}

/**
 * Get steps count from given angle in degrees.
 */
unsigned int GetStepsFromAngleDeg(double angleDeg,  unsigned int revSteps, unsigned int microstep)
{
    return ((angleDeg / 360) * revSteps * microstep);
}

/**
 * Get steps count from given angle in radians.
 */
unsigned int GetStepsFromAngleRad(double angleRad,  unsigned int revSteps, unsigned int microstep)
{
    return ((angleRad / (2 * M_PI)) * revSteps * microstep);
}

