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
const int effectiveMaxThrottle = maxThrottle - 200;
const int armLED = A1;
const int readyLED = A0;
const int armButton = 2;
boolean armed = false;

struct SensorError {
  float pitch;
  float roll;
};
SensorError sensorError = {0, 0};
boolean calibrating = true;

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

void setup () {
  Serial.begin(115200);

  // Initialize sensors
  if (!accel.begin()) {
    Serial.println(F("No LSM303"));
    while(true);
  }
  if (!mag.begin()) {
    Serial.println(F("No LSM303"));
    while(true);
  }
  if (!bmp.begin()) {
    Serial.println(F("No BMP180"));
    while(true);
  }
  
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
  // Calibrate accelerometer
  const unsigned short trials = 20;
  for (int trial = 0; trial < trials; trial++) {
    sensors_vec_t sensorData = getOrientation();
    sensorError.pitch += (float)sensorData.pitch;
    sensorError.roll += (float)sensorData.roll;
    delay(20);
  }
  sensorError.pitch /= trials;
  sensorError.roll /= trials;
  calibrating = false;
  
  digitalWrite(armLED, HIGH);
}

void loop () {
  setThrottleAll(minThrottle);
  
  sensors_vec_t orientation = getOrientation();
  Serial.print(F("Roll: "));
  Serial.print(orientation.roll);
  Serial.print(F("; Pitch: "));
  Serial.print(orientation.pitch);
  Serial.print(F("; Heading: "));
  Serial.println(orientation.heading);
}

sensors_vec_t getOrientation() {
  sensors_event_t accelEvent;
  sensors_event_t magEvent;
  sensors_vec_t   orientation;
  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accelEvent);
  mag.getEvent(&magEvent);
  dof.fusionGetOrientation(&accelEvent, &magEvent, &orientation);
  float roll = -orientation.roll;
  orientation.roll = -orientation.pitch;
  orientation.pitch = roll;
  if (!calibrating) {
    orientation.pitch -= sensorError.pitch;
    orientation.roll -= sensorError.roll;
  }
  orientation.heading = -orientation.heading;
  return orientation;
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
