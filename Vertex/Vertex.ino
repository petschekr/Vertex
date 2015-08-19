#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include "Config.h"

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

struct SensorError {
  float pitch;
  float roll;
  float elevation;
};
SensorError sensorError = {0, 0, 0};
boolean calibrating = true;

struct Attitude {
  double pitch;
  double roll;
  double yaw;
};
Attitude attitude;
Attitude desiredAttitude = {0, 0, 0};
unsigned long lastAttitudeUpdate = 0;

// Set up PID controllers
double pitchPID, rollPID, yawPID, altitudePID;
PID pitchPIDController(&attitude.pitch, &pitchPID, &desiredAttitude.pitch, PITCH_KP, PITCH_KI, PITCH_KD, REVERSE);
//PID rollPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//PID yawPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//PID altitudePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

void setup () {
  Serial.begin(115200);

  // Initialize sensors
  if (!accel.begin() || !mag.begin() || !bmp.begin() || !gyro.begin()) {
    fatalError();
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
  sensors_vec_t sensorData;
  setThrottleAll(1200);
  // Wait for propellers to get up to speed
  delay(1000);
  for (int trial = 0; trial < trials; trial++) {
    sensorData = getOrientation();
    sensorError.pitch += (float)sensorData.pitch;
    sensorError.roll += (float)sensorData.roll;
    sensorError.elevation += getAltitude();
    delay(20);
  }
  sensorError.pitch /= trials;
  sensorError.roll /= trials;
  sensorError.elevation /= trials;
  calibrating = false;

  sensorData = getOrientation();
  attitude.pitch = sensorData.pitch;
  attitude.roll = sensorData.roll;
  attitude.yaw = sensorData.heading;
  desiredAttitude.yaw = sensorData.heading;
  lastAttitudeUpdate = millis();

  pitchPIDController.SetMode(AUTOMATIC);
  pitchPIDController.SetOutputLimits(-50, 50);
  
  digitalWrite(armLED, HIGH);
}

void loop () {
  const int testThrottle = 20;
  int rawThrottle = map(testThrottle, 0, 100, minThrottle, maxThrottle);
  
  sensors_vec_t orientation = getOrientation();
  updateAttitude();
  //float altitude = getAltitude();
  
  pitchPIDController.Compute();

  setThrottle(1, rawThrottle - pitchPID);
  setThrottle(2, rawThrottle + pitchPID);
  setThrottle(3, rawThrottle - pitchPID);
  setThrottle(4, rawThrottle + pitchPID);
  /*
  int m0_val = throttle + PIDroll_val + PIDpitch_val + PIDyaw_val; 
  int m1_val = throttle - PIDroll_val + PIDpitch_val - PIDyaw_val; 
  int m2_val = throttle + PIDroll_val - PIDpitch_val - PIDyaw_val; 
  int m3_val = throttle - PIDroll_val - PIDpitch_val + PIDyaw_val; 
  */
  Serial.print("Throttle: ");
  Serial.print(rawThrottle);
  Serial.print(";\tpitchPID: ");
  Serial.print(pitchPID);
  Serial.print(";\tR Pitch: ");
  Serial.print(orientation.pitch);
  Serial.print("; \tA Pitch: ");
  Serial.println(attitude.pitch);
}

void updateAttitude() {
  unsigned int elapsedTime = millis() - lastAttitudeUpdate;
  float elapsedSeconds = elapsedTime * 0.001;
  sensors_vec_t orientation = getOrientation();
  
  // X corresponds to pitch, Y corresponds to roll, Z corresponds to yaw
  sensors_vec_t gyroData = getGyro();
  // Convert rad/s to degrees/s to match accelerometer's units
  gyroData.x *= (180 / PI);
  gyroData.y *= (180 / PI);
  gyroData.z *= (180 / PI);

  // angle = 0.98 * (angle + gyroData * dt) + 0.02 * (accData)
  attitude.pitch = complementaryFilter(attitude.pitch, gyroData.x, elapsedSeconds, orientation.pitch);
  attitude.roll = complementaryFilter(attitude.roll, gyroData.y, elapsedSeconds, orientation.roll);
  // Immediately correct when compass switches between -180 and 180 degrees
  if (abs(orientation.heading - attitude.yaw) > 100) {
    attitude.yaw = orientation.heading;
  }
  else {
    attitude.yaw = complementaryFilter(attitude.yaw, gyroData.z, elapsedSeconds, orientation.heading);
  }
  lastAttitudeUpdate = millis();
  delay(5);
}
double complementaryFilter (double angle, float gyroData, float elapsedSeconds, double accData) {
  return 0.98 * (angle + gyroData * elapsedSeconds) + 0.02 * accData;
}

#define BUFFER_SIZE 15
int pitchBufferIndex;
float pitchBuffer[BUFFER_SIZE];
float pitchBufferSum;
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
    
    pitchBufferSum -= pitchBuffer[pitchBufferIndex];
    pitchBuffer[pitchBufferIndex] = orientation.pitch;
    pitchBufferSum += orientation.pitch;
    pitchBufferIndex++;
    if (pitchBufferIndex >= BUFFER_SIZE)
      pitchBufferIndex = 0;
    orientation.pitch = pitchBufferSum / BUFFER_SIZE;
  }
  orientation.heading = -orientation.heading;
  return orientation;
}
sensors_vec_t getGyro() {
  sensors_event_t gyroEvent;
  gyro.getEvent(&gyroEvent);
  gyroEvent.gyro.x = -gyroEvent.gyro.x;
  gyroEvent.gyro.y = -gyroEvent.gyro.y;
  gyroEvent.gyro.z = -gyroEvent.gyro.z;
  return gyroEvent.gyro;
}
float getAltitude() {
  sensors_event_t baroEvent;
  bmp.getEvent(&baroEvent);
  if (calibrating) {
    return bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, baroEvent.pressure);
  }
  else {
    return bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, baroEvent.pressure) - sensorError.elevation;
  }
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
  digitalWrite(armLED, HIGH);
  digitalWrite(readyLED, HIGH);
  while (true) {}
}
