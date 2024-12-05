//***3RPS Parallel Manipulator Ball Balancer Code BY Aaed Musa**
//--------------------------------------------------------------

// Constants
#define A 0
#define B 1
#define C 2

// Include libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TouchScreen.h>
#include <stdint.h>
#include <math.h>

// inv_kinematics class
class inv_kinematics {
  public:
    inv_kinematics(double _d, double _e, double _f, double _g);
    double theta(int leg, double h_z, double n_x, double n_y);
  private:
    double d, e, f, g;  // Parameters
};

// Global Variables
inv_kinematics::inv_kinematics(double _d, double _e, double _f, double _g) {
  d = _d;
  e = _e;
  f = _f;
  g = _g;
}

double inv_kinematics::theta(int leg, double h_z, double n_x, double n_y) {
  double n_mag = sqrt(pow(n_x, 2) + pow(n_y, 2) + 1);
  n_x /= n_mag;
  n_y /= n_mag;
  double n_z = 1 / n_mag;

  double x, y, z, mag, angle;
  switch (leg) {
    case A: // Leg A
      y = d + (e / 2) * (1 - (pow(n_x, 2) + 3 * pow(n_z, 2) + 3 * n_z) /
          (n_z + 1 - pow(n_x, 2) + (pow(n_x, 4) - 3 * pow(n_x, 2) * pow(n_y, 2)) /
          ((n_z + 1) * (n_z + 1 - pow(n_x, 2)))));
      z = h_z + e * n_y;
      mag = sqrt(pow(y, 2) + pow(z, 2));
      angle = acos(y / mag) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
    case B: // Leg B
      x = (sqrt(3) / 2) * (e * (1 - (pow(n_x, 2) + sqrt(3) * n_x * n_y) / (n_z + 1)) - d);
      y = x / sqrt(3);
      z = h_z - (e / 2) * (sqrt(3) * n_x + n_y);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = acos((sqrt(3) * x + y) / (-2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
    case C: // Leg C
      x = (sqrt(3) / 2) * (d - e * (1 - (pow(n_x, 2) - sqrt(3) * n_x * n_y) / (n_z + 1)));
      y = -x / sqrt(3);
      z = h_z + (e / 2) * (sqrt(3) * n_x - n_y);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = acos((sqrt(3) * x - y) / (2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
  }
  return (angle * (180 / PI)); // Convert to degrees
}

// Global objects and variables
inv_kinematics inv_kinematics(2, 3.125, 1.75, 3.669291339);
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);
AccelStepper stepperA(1, 1, 2);
AccelStepper stepperB(1, 3, 4);
AccelStepper stepperC(1, 5, 6);
MultiStepper steppers;

long int pos[3];
int ENA = 0;
double angle_origin = 206.662752199;
double speed[3] = {0, 0, 0}, speed_previous[3], ks = 20;
double x_offset = 500, y_offset = 500;
double k_p= 4E-4, k_i= 2E-6, k_d = 7E-3;
double error[2] = {0, 0}, error_previous[2], integr[2] = {0, 0}, deriv[2] = {0, 0}, out[2];
long time_i;
double angle_to_step = 3200 / 360;
bool detected = 0;

void setup() {
  Serial.begin(115200);
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  delay(1000);
  digitalWrite(ENA, LOW);
  moveTo(4.25, 0, 0);
  steppers.runSpeedToPosition();
}

void loop() {
  PID(0, 0);
}

void moveTo(double h_z, double n_x, double n_y) {
  if (detected) {
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angle_origin - inv_kinematics.theta(i, h_z, n_x, n_y)) * angle_to_step);
    }
    stepperA.setMaxSpeed(speed[A]);
    stepperB.setMaxSpeed(speed[B]);
    stepperC.setMaxSpeed(speed[C]);
    stepperA.setAcceleration(speed[A] * 30);
    stepperB.setAcceleration(speed[B] * 30);
    stepperC.setAcceleration(speed[C] * 30);
    stepperA.moveTo(pos[A]);
    stepperB.moveTo(pos[B]);
    stepperC.moveTo(pos[C]);
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
      integr[i] += error[i] + error_previous[i];
      deriv[i] = error[i] - error_previous[i];
      deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];
      out[i] = k_p* error[i] + k_i* integr[i] + k_d * deriv[i];
      out[i] = constrain(out[i], -0.25, 0.25);
    }
    for (int i = 0; i < 3; i++) {
      speed_previous[i] = speed[i];
      speed[i] = (i == A) * stepperA.currentPosition() + (i == B) * stepperB.currentPosition() + (i == C) * stepperC.currentPosition();
      speed[i] = abs(speed[i] - pos[i]) * ks;
      speed[i] = constrain(speed[i], speed_previous[i] - 200, speed_previous[i] + 200);
      speed[i] = constrain(speed[i], 0, 1000);
    }
    Serial.println((String) "X OUT = " + out[0] + "   Y OUT = " + out[1] + "   Speed A: " + speed[A]);
  } else {
    delay(10);
    TSPoint p = ts.getPoint();
    if (p.x == 0) {
      detected = 0;
    }
  }
  time_i = millis();
  while (millis() - time_i < 20) {
    moveTo(4.25, -out[0], -out[1]);
  }
}
