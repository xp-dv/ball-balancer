#include <AccelStepper.h>
#include <MultiStepper.h>

// Pin Assignment
enum pins {
  ENABLE_PIN = 2, // TMC2208 Enable (EN) Pin
  STEP_PIN_A = 3, // Step-Signal Input A
  DIR_PIN_A = 4,  // Direction-Signal Input A
  STEP_PIN_2 = 5, // Step-Signal Input 2
  DIR_PIN_2 = 6,  // Direction-Signal Input 2
  STEP_PIN_3 = 7, // Step-Signal Input 3
  DIR_PIN_3 = 8,  // Direction-Signal Input 3
};

// User Constants
#define HOME_ANGLE 206 // Do not go lower than 200
#define SPEED 200 // Do not exceed 800
#define STEP_SIZE 25
#define NUMBER_OF_STEPS 20

// Generate stepper motor instances
const int DRIVER_TYPE = 1;
AccelStepper stepperA(DRIVER_TYPE, STEP_PIN_A, DIR_PIN_A);  // (driver type, STEP_PIN, DIR_PIN)
AccelStepper stepperB(DRIVER_TYPE, STEP_PIN_2, DIR_PIN_2);  // (driver type, STEP_PIN, DIR_PIN)
AccelStepper stepperC(DRIVER_TYPE, STEP_PIN_3, DIR_PIN_3);  // (driver type, STEP_PIN, DIR_PIN)
// Generate MultiStepper instance
MultiStepper steppers;

// Stepper home position array
long pos[3] = {HOME_ANGLE, HOME_ANGLE, HOME_ANGLE};

void setup() {
  // Serial.begin(9600);
  //Set iniial maximum speed value for the steppers (steps/sec)
  stepperA.setMaxSpeed(SPEED);
  stepperB.setMaxSpeed(SPEED);
  stepperC.setMaxSpeed(SPEED);
  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);
  // TMC2208 driver enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  delay(1000); // Delay for platform reset
  // Move to home position
  steppers.moveTo(pos);
  // Block CPU until all steppers reach their target position
  steppers.runSpeedToPosition();
}

void loop() {
  const int NUMBER_OF_MOTORS = 3;
  for (int i = 0; i < NUMBER_OF_STEPS; ++i) {
    for (int k = 0; k < NUMBER_OF_MOTORS; ++k) {
      pos[k] += STEP_SIZE;
    }
    steppers.moveTo(pos);
    steppers.runSpeedToPosition();
  }
  for (int i = 0; i < NUMBER_OF_STEPS; ++i) {
    for (int k = 0; k < NUMBER_OF_MOTORS; ++k) {
      pos[k] -= STEP_SIZE;
    }
    steppers.moveTo(pos);
    steppers.runSpeedToPosition();
  }
}
