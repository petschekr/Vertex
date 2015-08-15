#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;
/*
 Motor setup
 4       1
  \     /
   \___/
   /   \
  /     \
 3       2
*/

const int maxThrottle = 2000;
const int minThrottle = 1000;
const int armLED = A1;
const int readyLED = A0;
const int armButton = 2;
boolean armed = false;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

void setup () {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) {};
#endif
  ESC1.attach(5);
  ESC2.attach(6);
  ESC3.attach(7);
  ESC4.attach(8);
  setThrottleAll(0);

  pinMode(armLED, OUTPUT);
  pinMode(readyLED, OUTPUT);
  pinMode(armButton, INPUT);
  digitalWrite(armLED, LOW);
  digitalWrite(readyLED, HIGH);
  // Set up PID controllers
  PID pitchPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  PID rollPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  PID yawPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  PID altitudePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

  setThrottleAll(minThrottle);
  // Wait until armed
  while (digitalRead(armButton) != HIGH) {
    delay(10);
  }
  // Arm button pressed, arm the quadcopter
  digitalWrite(readyLED, LOW);
  unsigned int startTime = (int)millis();
  boolean armLEDOn = false;
  // Blink the LED while we wait for the ESCs to arm for 5 seconds
  while ((int)millis() - startTime < 5000) {
    if (armLEDOn) {
      digitalWrite(armLED, LOW);
      armLEDOn = !armLEDOn;
      delay(250);
    }
    else {
      digitalWrite(armLED, HIGH);
      delay(250);
      armLEDOn = !armLEDOn;
    }
  }
  digitalWrite(armLED, HIGH);
}

void loop () {
  
}

void setThrottleAll (int throttle) {
  setThrottle(1, throttle);
  setThrottle(2, throttle);
  setThrottle(3, throttle);
  setThrottle(4, throttle);
}
void setThrottle (int index, int throttle) {
  if (index == 1)
    ESC1.writeMicroseconds(throttle);
  if (index == 2)
    ESC2.writeMicroseconds(throttle);
  if (index == 3)
    ESC3.writeMicroseconds(throttle);
  if (index == 4)
    ESC4.writeMicroseconds(throttle);
}
void fatalError () {
  setThrottleAll(0);
  while (true) {
    digitalWrite(armLED, LOW);
    digitalWrite(readyLED, LOW);
    delay(250);
    digitalWrite(armLED, HIGH);
    digitalWrite(readyLED, HIGH);
    delay(250);
  }
}
