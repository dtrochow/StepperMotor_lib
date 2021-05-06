#include "inc/StepperMotor.h"

//absolute_time_t abs_time;
uint64_t abs_time;

StepperMotor_t *stepperMotors[MAX_MOTOR_QUANTITY];
int motorsQuantity = 0;

/**
 * Make one step in chosen direction.
 * It is made by setting low state on CLK driver pin. 
 * @TODO Describe how to set STEP on stepper motor driver (include description to differ drivers)
 */
void MakeStep(bool dir, StepperMotor_t *motor)
{
    gpio_put(motor->dirPin, dir);
    gpio_put(motor->stepPin, 0);
    sleep_us(3);
    gpio_put(motor->stepPin, 1);
}

/**
 * Motor initialization.
 * @TODO Describe what this inicializzation doing.
 */
void InitMotor(StepperMotor_t *motor, uint stepPin_, uint dirPin_, uint enPin_,
                double current_, double nVoltage_, unsigned int revSteps_, 
                unsigned int microstep_, unsigned int decaySetting_, double nTorque_,
                float gearRatio_)
{
    motor->stepPin = stepPin_;
    motor->dirPin = dirPin_;
    motor->enPin = enPin_;
    motor->current = current_;
    motor->microstep = microstep_;
    motor->decaySetting = decaySetting_;
    motor->nVoltage = nVoltage_;
    motor->nTorque = nTorque_;
    motor->gearRatio = gearRatio_;
    motor->revSteps = revSteps_;
    motor->speed_rad = 0.1;   //Set initial motor speed to 0 rad/s
    motor->speed_deg = 0;   //Set initial motor speed to 0 deg/s
    motor->degPerStep = 360.0/(motor->revSteps * motor->gearRatio * motor->microstep);
    motor->stepPerDeg = 1/motor->degPerStep;
    motor->radPerStep = (2*M_PI)/(motor->revSteps * motor->gearRatio * motor->microstep);
    motor->stepPerRad = 1/motor->radPerStep;
    motor->posJoint_deg = 0;
    motor->posJoint_rad = 0;
    motor->posMotor_deg = 0;
    motor->posMotor_rad = 0;
    motor->stepTime = 0;
    motor->rotationMode = MOVE_WITH_SET_SPEED;

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

    SetMotorSpeedRad(motor, motor->speed_rad);

    stepperMotors[motorsQuantity] = motor; //Add motor pointer to motors array
    motorsQuantity ++;
}

/**
 * Rotete stepper motor by given angle in degrees.
 */
void RotateMotor_angleDeg(StepperMotor_t *motor, double angleDeg, bool dir)
{
    unsigned int steps = GetStepsFromAngleDeg(angleDeg, motor->revSteps, motor->microstep);
    for(int i = 0; i < steps; i++)
    {
        MakeStep(dir, motor);
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
        MakeStep(dir, motor);
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
        MakeStep(dir, motor);
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

/**
 * Set stepper motor angular velocity [deg/s].
 */
void SetMotorSpeedDeg(StepperMotor_t *motor, double speed)
{
    motor->speed_deg = speed;
    motor->speed_rad = (speed / 360) * (2*M_PI);
    motor->deltaT = (motor->degPerStep / motor->speed_deg) * 1000000;
}

/**
 * Set stepper motor angular velocity [rad/s].
 */
void SetMotorSpeedRad(StepperMotor_t *motor, double speed)
{
    motor->speed_rad = speed;
    motor->speed_deg = (speed / 2*M_PI) * 360;
    motor->deltaT = (motor->radPerStep / motor->speed_rad) * 1000000;
}

/**
 * 
 * 
 */
void ControlLoop()
{
    for(int i = 0; i < motorsQuantity; i++)
    {   
        abs_time = time_us_64();
        switch(stepperMotors[i]->rotationMode)
        {
            case MOVE_BY_ANGLE:
            // Check the error (setAngle - actualAngle)

            break;

            case MOVE_WITH_SET_SPEED:
            // Do not check the error and continously rotate the shaft.
                if((abs_time - stepperMotors[i]->stepTime) >= stepperMotors[i]->deltaT)
                {
                    MakeStep(CLOCKWISE, stepperMotors[i]);
                    stepperMotors[i]->stepTime = abs_time;
                    //Need to update current angle
                }
            break;

            case MOVE_WITH_SET_SPEED_FOR_TIME:
            // Rotate motor shaft by given time in some variable in stepperMotor structure.

            break;

            case DISABLED:
            // Do not move the motor shaft, keep torque on shaft.
                
            break;

            case DISABLED_NO_TORQUE:
            // Do not move the motor shaft, do not keep torque on shaft.

            break;
        }
    }
}

/**
 * 
 * 
 */
void UpdatePosAfterStep(StepperMotor_t *motor)
{
    
}