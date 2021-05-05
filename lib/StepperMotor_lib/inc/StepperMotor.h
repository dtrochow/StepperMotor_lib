#define CLOCKWISE           1
#define COUNTER_CLOCKWISE   0

#define MOTOR_QUANTITY      1

#include "pico/stdlib.h"
#include <math.h>
#include "pico/types.h"

typedef struct StepperMotor_t{
    uint stepPin;               //Clock pin.
    uint dirPin;                //Rotation direcion pin.
    uint enPin;                 //Enable pin.
    double current;             //Nominal motor current. [A]
    unsigned int microstep;     //Microstepping mode e.g.   [1] - microstepping disabled
                                //                          [2] - microstepping 1/2
                                //                          [4] - microstepping 1/4
    float gearRatio;            //Gear ration between motor shaft and joint. (driven/driving)
    unsigned int decaySetting;  //Decay setting value. [%]
    unsigned int revSteps;      //Steps by full rotary shaft revolution. (e.g. 200)
    double nVoltage;            //Nominal power supply voltage. [V]
    double nTorque;             //Nominal motor torque. [Nm]
    double speed_rad;           //Speed of joint driven by motor. [rad/s]
    double speed_deg;           //Speed of joint driven by motor. [deg/s]
    double posMotor_deg;        //Motor shaft angle position. [degrees] 
    double posMotor_rad;        //Motor shaft angle position. [radians]
    double posJoint_deg;        //Position of joint driven by motor. [degrees]
    double posJoint_rad;        //Position of joint driven by motor. [radians]
    double degPerStep;          //Joint rotation angle make by one step. [degrees]
    double stepPerDeg;          //Count of steps needed to make one degree joint rotation.
    double radPerStep;          //Joint rotation angle make by one step. [radians]
    double stepPerRad;          //Count of steps needed to make one radian joint rotation.
    double deltaT;              //Count of microseconds between steps (depends on motor speed).
}StepperMotor_t;

/**
 * Make one step in chosen direction.
 * 
 * @param dir       Rotation direction. 1 - clockwise, 0 - counter clockwise.
 * @param motor     Stepper motor structure with all motor parameters.
 */
void MakeStep(bool dir, StepperMotor_t motor);

/**
 * Motor initialization.
 * 
 * @param motor             Pointer to stepper motor structure with all motor parameters.
 * @param stepPin_          Pin connected to CLK (clock) input of stepper motor controler.  
 * @param dirPin_           Pin connected to DIR (direction) input of stepper motor controler.
 * @param enPin_            Pin connected to ENA (enabled) input of stepper motor controler.
 * @param current_          Nominal current set in driver (nominal current of stepper motor).
 * @param nVoltage_         Nominal stepper motor voltage.
 * @param nTorque_          Nominal motor torque.
 * @param revSteps_         Full revolution steps count.
 * @param microstep_        Microstep setting set on stepper motor driver.
 * @param decaySetting_     Decay setting on stepper motor driver (if possible).
 */
void InitMotor(StepperMotor_t *motor, uint stepPin_, uint dirPin_, uint enPin_,
                double current_, double nVoltage_, unsigned int revSteps_, 
                unsigned int microstep_, unsigned int decaySetting_, double nTorque_);

/**
 * Rotete stepper motor by given angle in degrees.
 * 
 * @param motor         Pointer to stepper motor structure with all motor parameters. 
 * @param angleDeg      Rotation angle in degrees. 
 * @param dir           Rotation direction.
 */
void RotateMotor_angleDeg(StepperMotor_t *motor, double angleDeg, bool dir);

/**
 * Rotate stepper motor by given angle in radians.
 * 
 * @param motor         Pointer to stepper motor structure with all motor parameters. 
 * @param angleRad      Rotation angle in radians.
 * @param dir           Rotation direction.
 */
void RotateMotor_angleRad(StepperMotor_t *motor, double angleRad, bool dir);

/**
 * Rotate stepper motor by given steps count.
 * 
 * @param motor         Pointer to stepper motor structure with all motor parameters. 
 * @param steps         Amount of steps rotate by motor.
 * @param dir           Rotation direction.
 */
void RotateMotor_steps(StepperMotor_t *motor, double steps, bool dir);

/**
 * Get steps count from given angle in degrees.
 * 
 * @param angleDeg      Rotation angle in radians.
 * @param revSteps      Full revolution steps count.
 * @param microstep     Microstep setting set on stepper motor driver.
 * 
 * @return              Amount of steps.
 */
unsigned int GetStepsFromAngleDeg(double angleDeg,  unsigned int revSteps, unsigned int microstep);

/**
 * Get steps count from given angle in radians.
 * 
 * @param angleRad      Rotation angle in degrees. 
 * @param revSteps      Full revolution steps count.
 * @param microstep     Microstep setting set on stepper motor driver.
 * 
 * @return              Amount of steps.
 */
unsigned int GetStepsFromAngleRad(double angleRad,  unsigned int revSteps, unsigned int microstep);

/**
 * Set stepper motor angular velocity [deg/s].
 * 
 * @param motor     Pointer to stepper motor structure with all motor parameters.  
 * @param speed     Stepper motor angular velocity. [deg/s]
 */
void SetMotorSpeedDeg(StepperMotor_t *motor, double speed);

/**
 * Set stepper motor angular velocity [rad/s].
 * 
 * @param motor     Pointer to stepper motor structure with all motor parameters.  
 * @param speed     Stepper motor angular velocity. [rad/s]
 */
void SetMotorSpeedRad(StepperMotor_t *motor, double speed);
