#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TouchScreen.h>
#include <stdint.h>
#include <math.h>

//* User Definitions
// PID
#define KP 17E-5
#define KI 1E-7
#define KD 4E-3
#define KS 15
// Misc
#define HOME_HEIGHT 4.25
#define BAUD_RATE 115200

// Pin Assignment
enum pins {
  // TMC2208 Stepper Motor Drivers
  ENABLE_PIN = 2, // Enable (EN) Pin
  STEP_PIN_A = 3, // Stepper A Step Signal (STEP)
  DIR_PIN_A  = 4, // Stepper A Direction Signal (DIR)
  STEP_PIN_B = 5, // Stepper B Step Signal (STEP)
  DIR_PIN_B  = 6, // Stepper B Direction Signal (DIR)
  STEP_PIN_C = 7, // Stepper C Step Signal (STEP)
  DIR_PIN_C  = 8,  // Stepper C Direction Signal (DIR)
  Y_PLUS_PIN  = A0, // Resistive Touch Screen
  X_PLUS_PIN  = A1, // Resistive Touch Screen
  Y_MINUS_PIN = A2, // Resistive Touch Screen
  X_MINUS_PIN = A3, // Resistive Touch Screen
};

//* Private Definitions
// Leg Assignment
enum legs {
  LEG_A,
  LEG_B,
  LEG_C,
};

// Declare inverse kinematics class
class inv_kinematics {
  public:
    inv_kinematics(double _d, double _e, double _f, double _g);
    double theta(int leg, double h_z, double n_x, double n_y);
  private:
    double d, e, f, g;  // Parameters
};

// Define inverse kinematics class primary function
inv_kinematics::inv_kinematics(double _d, double _e, double _f, double _g) {
  d = _d;
  e = _e;
  f = _f;
  g = _g;
}

// Define inverse kinematics class theta
double inv_kinematics::theta(int leg, double h_z, double n_x, double n_y) {
  double n_mag = sqrt(pow(n_x, 2) + pow(n_y, 2) + 1);
  n_x /= n_mag;
  n_y /= n_mag;
  double n_z = 1 / n_mag;

  double x, y, z, mag, angle;
  switch (leg) {
    case LEG_A:
      y = d + (e / 2) * (1 - (pow(n_x, 2) + 3 * pow(n_z, 2) + 3 * n_z) /
          (n_z + 1 - pow(n_x, 2) + (pow(n_x, 4) - 3 * pow(n_x, 2) * pow(n_y, 2)) /
          ((n_z + 1) * (n_z + 1 - pow(n_x, 2)))));
      z = h_z + e * n_y;
      mag = sqrt(pow(y, 2) + pow(z, 2));
      angle = acos(y / mag) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
    case LEG_B:
      x = (sqrt(3) / 2) * (e * (1 - (pow(n_x, 2) + sqrt(3) * n_x * n_y) / (n_z + 1)) - d);
      y = x / sqrt(3);
      z = h_z - (e / 2) * (sqrt(3) * n_x + n_y);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = acos((sqrt(3) * x + y) / (-2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
    case LEG_C:
      x = (sqrt(3) / 2) * (d - e * (1 - (pow(n_x, 2) - sqrt(3) * n_x * n_y) / (n_z + 1)));
      y = -x / sqrt(3);
      z = h_z + (e / 2) * (sqrt(3) * n_x - n_y);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = acos((sqrt(3) * x - y) / (2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
  }
  return (angle * (180 / PI)); // Convert to degrees
}

// Create inverse kinematics instance
inv_kinematics inv_kinematics(2, 3.125, 1.75, 3.669291339);

// Create touch screen instance
TouchScreen ts = TouchScreen(X_PLUS_PIN, Y_PLUS_PIN, X_MINUS_PIN, Y_MINUS_PIN, 0);

// Create stepper motor instances
const int DRIVER_TYPE = 1; // Normal
// AccelStepper stepper_name(DRIVER_TYPE, STEP_PIN, DIR_PIN);
AccelStepper stepperA(DRIVER_TYPE, STEP_PIN_A, DIR_PIN_A);
AccelStepper stepperB(DRIVER_TYPE, STEP_PIN_B, DIR_PIN_B);
AccelStepper stepperC(DRIVER_TYPE, STEP_PIN_C, DIR_PIN_C);

// Create MultiStepper instance for AccelStepper instances
MultiStepper steppers;

// Global variables
long int pos[3];
double angle_origin = 206.662752199;
double speed[3] = {0, 0, 0};
double speed_previous[3];
double x_offset = 500;
double y_offset = 500;
double error[2] = {0, 0};
double error_previous[2];
double integral[2] = {0, 0};
double derivative[2] = {0, 0};
double out[2];
long time_i;
double angle_to_step = 3200 / 360;
bool detected = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  // Initialize steppers
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  // Set enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);
  delay(1000);
  digitalWrite(ENABLE_PIN, LOW);

  // Go to home position
  move(HOME_HEIGHT, 0, 0);
  steppers.runSpeedToPosition();
}

void loop() {
  PID(0, 0);
}

void move(double h_z, double n_x, double n_y) {
  if (detected) {
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angle_origin - inv_kinematics.theta(i, h_z, n_x, n_y)) * angle_to_step);
    }
    stepperA.setMaxSpeed(speed[LEG_A]);
    stepperB.setMaxSpeed(speed[LEG_B]);
    stepperC.setMaxSpeed(speed[LEG_C]);
    stepperA.setAcceleration(speed[LEG_A] * 30);
    stepperB.setAcceleration(speed[LEG_B] * 30);
    stepperC.setAcceleration(speed[LEG_C] * 30);
    stepperA.moveTo(pos[LEG_A]);
    stepperB.moveTo(pos[LEG_B]);
    stepperC.moveTo(pos[LEG_C]);
    stepperA.run();
    stepperB.run();
    stepperC.run();
  } else {
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angle_origin - inv_kinematics.theta(i, h_z, 0, 0)) * angle_to_step);
    }
    stepperA.setMaxSpeed(800);
    stepperB.setMaxSpeed(800);
    stepperC.setMaxSpeed(800);
    steppers.moveTo(pos);
    steppers.run();
  }
}

void PID(double setpoint_X, double setpoint_Y) {
  TSPoint p = ts.getPoint();
  if (p.x != 0) {
    detected = 1;
    for (int i = 0; i < 2; i++) {
      error_previous[i] = error[i];
      error[i] = (i == 0) * (x_offset - p.x - setpoint_X) + (i == 1) * (y_offset - p.y - setpoint_Y);
      integral[i] += error[i] + error_previous[i];
      derivative[i] = error[i] - error_previous[i];
      derivative[i] = isnan(derivative[i]) || isinf(derivative[i]) ? 0 : derivative[i];
      out[i] = KP * error[i] + KI * integral[i] + KD * derivative[i];
      out[i] = constrain(out[i], -0.25, 0.25);
    }
    for (int i = 0; i < 3; i++) {
      speed_previous[i] = speed[i];
      speed[i] = (i == LEG_A) * stepperA.currentPosition() + (i == LEG_B) * stepperB.currentPosition() + (i == LEG_C) * stepperC.currentPosition();
      speed[i] = abs(speed[i] - pos[i]) * KS;
      speed[i] = constrain(speed[i], speed_previous[i] - 200, speed_previous[i] + 200);
      speed[i] = constrain(speed[i], 0, 1000);
    }
    Serial.println((String) "X OUT = " + out[0] + "   Y OUT = " + out[1] + "   Speed A: " + speed[LEG_A]);
  } else {
    delay(10);
    TSPoint p = ts.getPoint();
    if (p.x == 0) {
      detected = 0;
    }
  }
  time_i = millis();
  while (millis() - time_i < 20) {
    move(4.25, -out[0], -out[1]);
  }
}
