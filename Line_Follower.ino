#include <QTRSensors.h>

#define leftMotor1 2
#define leftMotor2 4
#define leftMotorPWM  3

#define rightMotor1 0
#define rightMotor2 1
#define rightMotorPWM  5

#define SEN_1 6
#define SEN_2 7
#define SEN_3 8
#define SEN_4 9
#define SEN_5 10
#define SEN_6 11
#define SEN_7 12
#define SEN_8 13

#define BUTTON 2
#define BTN_THRES 800// >75% of 1023

#define Kp 0.065 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.65 // experiment to determine this, slowly increase the speeds and adjust this value. (Note: Kp < Kd)
#define Ki 0.015
#define rightMaxSpeed 180 // max speed of the robot
#define leftMaxSpeed 180 // max speed of the robot

#define rightBaseSpeed 120 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 120  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   255     // emitter is controlled by digital pin 2

#define MOTOR_STEPS 4
#define STEP_THRESHOLD 100
inline void drive(int leftMotorPin, int rightMotorPin,
                  int leftIntensity, int rightIntensity,
                  int *lastLeftIntensity, int *lastRightIntensity) {
  int leftMotorStep;
  int rightMotorStep;
  // If intensity difference is greater than STEP_THRESHOLD, do MOTOR_STEPS intensity transitions
  if (abs(leftIntensity - *lastLeftIntensity) > STEP_THRESHOLD) {
    leftMotorStep = (leftIntensity - *lastLeftIntensity) / MOTOR_STEPS;
  } else {
    *lastLeftIntensity = leftIntensity;
    leftMotorStep = 0;
  }
  if (abs(rightIntensity - *lastRightIntensity) > STEP_THRESHOLD) {
     rightMotorStep = (rightIntensity - *lastRightIntensity) / MOTOR_STEPS;
  } else {
    *lastRightIntensity = rightIntensity;
    rightMotorStep = 0;
  }
  for (int cnt = 0; cnt < MOTOR_STEPS; cnt++) {
    *lastLeftIntensity += leftMotorStep;
    *lastRightIntensity += rightMotorStep;
    analogWrite(leftMotorPin, *lastLeftIntensity);
    analogWrite(rightMotorPin, *lastRightIntensity);
    delay(2);
  }
}

void driveMotors(int leftIntensity, int rightIntensity, int *lastLeftIntensity, int *lastRightIntensity) {
  int leftDirection = (leftIntensity >= 0) ? 1 : -1;
  int rightDirection = (rightIntensity >= 0) ? 1 : -1;
  if (leftDirection >= 0) {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
  } else {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    leftIntensity = -leftIntensity;
  }
  if (rightDirection >= 0) {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  } else {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    rightIntensity = -rightIntensity;
  }
  drive(leftMotorPWM, rightMotorPWM, leftIntensity, rightIntensity, lastLeftIntensity, lastRightIntensity);
}

QTRSensorsRC qtrrc((unsigned char[]) {
  SEN_1, SEN_2, SEN_3, SEN_4, SEN_5, SEN_6, SEN_7, SEN_8
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19

void setup() {
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);

  int i;
  for (int i = 0; i < 100; i++) {
    qtrrc.calibrate();
    delay(20);
  }
  while (analogRead(BUTTON) < BTN_THRES) {
    delay(500);
  }
  delay(1000);
}

int lastError = 0;
int integral = 0;
int lastRightMotorSpeed = 0;
int lastLeftMotorSpeed = 0;

unsigned int sensors[8];
void loop() {
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 3500;

  int motorSpeed = Kp * error + Kd * (error - lastError) + Ki * integral;
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  bool rightDir = (rightMotorSpeed >= 0) ? 1 : -1;
  bool leftDir = (leftMotorSpeed >= 0) ? 1 : -1;

  // prevent the motor from going beyond max speed
  if (rightMotorSpeed * rightDir > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed * rightDir;
  if (leftMotorSpeed * leftDir > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed * leftDir;

  driveMotors(leftMotorSpeed, rightMotorSpeed, &lastLeftMotorSpeed, &lastRightMotorSpeed);
}
