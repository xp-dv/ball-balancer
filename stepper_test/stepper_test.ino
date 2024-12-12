#include <AccelStepper.h>
#include <MultiStepper.h>

// Pin Assignment
enum pins {
  // TMC2208 Stepper Motor Drivers
  ENABLE_PIN = 2, // Enable (EN) Pin
  STEP_PIN_A = 3, // Stepper A Step Signal (STEP)
  DIR_PIN_A  = 4, // Stepper A Direction Signal (DIR)
  STEP_PIN_B = 5, // Stepper B Step Signal (STEP)
  DIR_PIN_B  = 6, // Stepper B Direction Signal (DIR)
  STEP_PIN_C = 7, // Stepper C Step Signal (STEP)
  DIR_PIN_C  = 8, // Stepper C Direction Signal (DIR)
};

// User Constants
#define HOME_ANGLE 206 // Do not go lower than 200
#define SPEED 200 // Do not exceed 800
#define STEP_SIZE 25
#define NUMBER_OF_STEPS 20

// Create stepper motor instances
const int DRIVER_TYPE = 1; // Normal
// AccelStepper stepper_name(DRIVER_TYPE, STEP_PIN, DIR_PIN);
AccelStepper stepperA(DRIVER_TYPE, STEP_PIN_A, DIR_PIN_A);
AccelStepper stepperB(DRIVER_TYPE, STEP_PIN_B, DIR_PIN_B);
AccelStepper stepperC(DRIVER_TYPE, STEP_PIN_C, DIR_PIN_C);

// Create MultiStepper instance for AccelStepper instances
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
