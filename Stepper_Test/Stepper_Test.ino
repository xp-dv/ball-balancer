#include <AccelStepper.h>
#include <MultiStepper.h>

// Pin Assignment
enum pins {
  ENABLE_PIN,  // TMC2208 Enable (EN) Pin
  STEP_PIN_A = 5,  // Step-Signal Input A
  DIR_PIN_A = 2,   // Direction-Signal Input A
  STEP_PIN_2 = 6,  // Step-Signal Input 2
  DIR_PIN_2 = 4,   // Direction-Signal Input 2
  STEP_PIN_3 = 9,  // Step-Signal Input 3
  DIR_PIN_3 = 10,   // Direction-Signal Input 3
};

// Generate MultiStepper instance
const int DRIVER_TYPE = 1;
AccelStepper stepperA(DRIVER_TYPE, STEP_PIN_A, DIR_PIN_A);  // (driver type, STEP_PIN, DIR_PIN)
AccelStepper stepperB(DRIVER_TYPE, STEP_PIN_2, DIR_PIN_2);  // (driver type, STEP_PIN, DIR_PIN)
AccelStepper stepperC(DRIVER_TYPE, STEP_PIN_3, DIR_PIN_3);  // (driver type, STEP_PIN, DIR_PIN)
MultiStepper steppers;

// Initial stepper positions array
long pos[3] = {350, 350, 350};

void setup() {
  // Serial.begin(9600);
  //Set iniial maximum speed value for the steppers (steps/sec)
  stepperA.setMaxSpeed(150);
  stepperB.setMaxSpeed(150);
  stepperC.setMaxSpeed(150);
  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);
  //Enable pin
  pinMode(ENABLE_PIN, OUTPUT);    //define enable pin as output
  digitalWrite(ENABLE_PIN, LOW);  //sets the drivers on initially
  delay(1000);                    //small delay to allow the user to reset the platform
  //Movemement
  steppers.moveTo(pos);           // Calculates the required speed for all motors
  steppers.runSpeedToPosition();  // blocks until all steppers reach their target position
}

void loop() {
  for (int i = 0; i < 10; ++i) {
    for (int k = 0; k < 3; ++k) {
      pos[k] += 10;
    }
    steppers.moveTo(pos);           // Calculates the required speed for all motors
    steppers.runSpeedToPosition();  // blocks until all steppers reach their target position
  }
  for (int i = 0; i < 10; ++i) {
    for (int k = 0; k < 3; ++k) {
      pos[k] -= 10;
    }
    steppers.moveTo(pos);           // Calculates the required speed for all motors
    steppers.runSpeedToPosition();  // blocks until all steppers reach their target position
  }
}
