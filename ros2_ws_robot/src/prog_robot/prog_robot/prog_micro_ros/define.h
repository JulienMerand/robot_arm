#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Definition des pins
#define ENA_6 22
#define DIR_6 23
#define PUL_6 24

#define ENA_5 27
#define DIR_5 28
#define PUL_5 29

#define ENA_4 30
#define DIR_4 31
#define PUL_4 32

#define ENA_3 34
#define DIR_3 35
#define PUL_3 36

#define ENA_1 38
#define DIR_1 39
#define PUL_1 40

#define ENA_2B 44
#define DIR_2B 45
#define PUL_2B 46

#define ENA_2A 49
#define DIR_2A 50
#define PUL_2A 51

#define PIN_GRIPPER 52

// Déclaration des moteurs AccelStepper
AccelStepper stepper1(AccelStepper::DRIVER, PUL_1, DIR_1);
AccelStepper stepper2A(AccelStepper::DRIVER, PUL_2A, DIR_2A);
AccelStepper stepper2B(AccelStepper::DRIVER, PUL_2B, DIR_2B);
AccelStepper stepper3(AccelStepper::DRIVER, PUL_3, DIR_3);
AccelStepper stepper4(AccelStepper::DRIVER, PUL_4, DIR_4);
AccelStepper stepper5(AccelStepper::DRIVER, PUL_5, DIR_5);
AccelStepper stepper6(AccelStepper::DRIVER, PUL_6, DIR_6);

AccelStepper stepper[] = {stepper1, stepper2A, stepper2B, stepper3, stepper4, stepper5, stepper6};
MultiStepper steppers;

Servo gripper;
