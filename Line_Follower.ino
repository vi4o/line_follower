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
#define BTN_THRES 767 // 75% of 1023

#define Kp 0.04 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.3 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define Ki 0
#define rightMaxSpeed 120 // max speed of the robot
#define leftMaxSpeed 120 // max speed of the robot
#define rightBaseSpeed 70 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 70  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   255     // emitter is controlled by digital pin 2

void driveLeftMotor(int intensity) {
  int direction = (intensity >= 0) ? 1 : -1;
  if (direction >= 0) {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite(leftMotorPWM, intensity);
  } else {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, -intensity);
  }
}

void driveRightMotor(int intensity) {
  int direction = (intensity >= 0) ? 1 : -1;
  if (direction >= 0) {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(rightMotorPWM, intensity);
  } else {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, -intensity);
  }
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
  
  // driveLeftMotor(180);
  //driveRightMotor(100);

  int i;
  for (int i = 0; i < 100; i++) { // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
    /* comment this part out for automatic calibration
    if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
       turn_right();
     else
       turn_left(); */
    qtrrc.calibrate();
    delay(20);
  }
  //wait();
  while (analogRead(BUTTON) < BTN_THRES) {
    delay(500);
  }
  delay(1000);
}

int lastError = 0;
int integral = 0;

unsigned int sensors[8];
void loop() {
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 3500;

  integral = integral + error;
  int motorSpeed = Kp * error + Kd * (error - lastError) + Ki * integral;
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  bool rightDir = (rightMotorSpeed >= 0) ? 1 : -1;
  bool leftDir = (leftMotorSpeed >= 0) ? 1 : -1;
  if (rightMotorSpeed * rightDir > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed * rightDir; // prevent the motor from going beyond max speed
  if (leftMotorSpeed * leftDir > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed * leftDir; // prevent the motor from going beyond max speed

  driveRightMotor(rightMotorSpeed);
  driveLeftMotor(leftMotorSpeed);
}
