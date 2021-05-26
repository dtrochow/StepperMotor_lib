#define MAX_MOTOR_QUANTITY  8

#include "pico/stdlib.h"
#include <math.h>
#include "pico/types.h"

typedef enum {
    MOVE_BY_ANGLE,                  //Move motor shaft by given angle with set velocity.
    MOVE_CONTINOUSLY,               //Move motor shaft continously with set velocity.
    MOVE_FOR_TIME,                  //Move motor shaft for given time with set velocity.
    MOVE_TO_SET_VALUE,              //Move motor shaft to set value.
    DISABLED,                       //Motor disabled, with torque on the shaft.
    ENABLED,                        //Motor enabled, with torque on the shaft.
    DISABLED_NO_TORQUE              //Motor disabled without the torque on the shaft.  
} Mode_e;

typedef enum {
    COUNTER_CLOCKWISE,
    CLOCKWISE                     
} RotationDir_e;

typedef struct {
    double shaftDeg;        //Motor shaft angle position. [degrees] 
    double shaftRad;        //Motor shaft angle position. [radians]
    double jointDeg;        //Position of joint driven by motor. [degrees]
    double jointRad;        //Position of joint driven by motor. [radians]
} AngularPosition_t;

typedef struct {
    double rad;             //Speed of joint driven by motor. [rad/s]
    double deg;             //Speed of joint driven by motor. [deg/s]
} RotationSpeed_t;

typedef struct {
    double deg;             //Set value of shaft angular position.
    double rad;
} AngleSetValue_t;

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
    RotationSpeed_t speed;      //Speed of joint driven by motor.
    double degPerStep;          //Joint rotation angle make by one step. [degrees]
    double stepPerDeg;          //Count of steps needed to make one degree joint rotation.
    double radPerStep;          //Joint rotation angle make by one step. [radians]
    double stepPerRad;          //Count of steps needed to make one radian joint rotation.
    double deltaT;              //Count of microseconds between steps (depends on motor speed).
    uint64_t stepTime;          //Variable used to make steps with given speed.
    Mode_e rotationMode;        //Mode of stepper motor rotation.
    RotationDir_e dir;          //Motor shaft rotate direction.
    AngularPosition_t position; //Angular positions.
    volatile AngleSetValue_t setValue;   //Set value of shaft angular position.
} StepperMotor_t;

/**
 * Make one step in chosen direction.
 * 
 * @param motor     Stepper motor structure with all motor parameters.
 */
void MakeStep(StepperMotor_t *motor);

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
                unsigned int microstep_, unsigned int decaySetting_, double nTorque_,
                float gearRatio_);

/**
 * Rotete stepper motor by given angle in degrees.
 * 
 * @param motor         Pointer to stepper motor structure with all motor parameters. 
 * @param angleDeg      Rotation angle in degrees. 
 * @param dir           Rotation direction.
 */
void RotateMotor_angleDeg(StepperMotor_t *motor, double angleDeg, RotationDir_e dir);

/**
 * Rotate stepper motor by given angle in radians.
 * 
 * @param motor         Pointer to stepper motor structure with all motor parameters. 
 * @param angleRad      Rotation angle in radians.
 * @param dir           Rotation direction.
 */
void RotateMotor_angleRad(StepperMotor_t *motor, double angleRad, RotationDir_e dir);

/**
 * Rotate stepper motor by given steps count.
 * 
 * @param motor         Pointer to stepper motor structure with all motor parameters. 
 * @param steps         Amount of steps rotate by motor.
 * @param dir           Rotation direction.
 */
void RotateMotor_steps(StepperMotor_t *motor, double steps, RotationDir_e dir);

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

/**
 * Function controling all defined motors.
 * IMPORTANT: This function should be put in while loop or intrrupt callback, 
 *            which is refreshed more often then every 10us(?).
 */
void ControlLoop();


/**
 * Update motor position, after every step.
 * 
 * @param motor     Pointer to stepper motor structure with all motor parameters. 
 */
void UpdatePosAfterStep(StepperMotor_t *motor);

/**
 * Change rotate direction.
 * 
 * @param motor         Pointer to stepper motor structure with all motor parameters.  
 * @param dir           Rotation direction.
 */
void ChangeRotateDirection(StepperMotor_t *motor, RotationDir_e dir);