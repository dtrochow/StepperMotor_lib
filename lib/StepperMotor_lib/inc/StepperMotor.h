#include "pico/stdlib.h"

typedef struct StepperMotor_t{
    uint stepPin;               //Clock pin
    uint dirPin;                //Rotation direcion pin  
    uint enPin;                 //Enable pin
    double current;              //Nominal motor current in [A]
    unsigned int microstep;     //Microstepping mode e.g.   [1] - microstepping disabled
                                //                          [2] - microstepping 1/2
                                //                          [4] - microstepping 1/4
    unsigned int decaySetting;  //Decay setting value in [%]
    unsigned int revSteps;      //Steps by full rotary shaft revolution. (e.g. 200)
    double  nVoltage;           //Nominal power supply voltage [V]
}StepperMotor_t;

/**
 * Make one step in chosen direction.
 * 
 * @param dir       Rotation direction. 1 - clockwise, 0 - counter clockwise.
 * @param motor     Stepper motor structure with all motor parameters.
 */
void make_step(bool dir, StepperMotor_t motor);

/**
 * Motor initialization.
 * 
 * @param motor             Pointer to stepper motor structure with all motor parameters.
 * @param stepPin_          Pin connected to CLK (clock) input of stepper motor controler.  
 * @param dirPin_           Pin connected to DIR (direction) input of stepper motor controler.
 * @param enPin_            Pin connected to ENA (enabled) input of stepper motor controler.
 * @param current_          Nominal current set in driver (nominal current of stepper motor).
 * @param nVoltage_         Nominal stepper motor voltage.
 * @param revSteps_         Full revolution steps count.
 * @param microstep_        Microstep setting set on stepper motor driver.
 * @param decaySetting_     Decay setting on stepper motor driver (if possible).
 */
void init_motor(StepperMotor_t *motor, uint stepPin_, uint dirPin_, uint enPin_,
                double current_, double nVoltage_, unsigned int revSteps_, 
                unsigned int microstep_, unsigned int decaySetting_);

//void rotate_motor(StepperMotor_t *motor, double angle, )