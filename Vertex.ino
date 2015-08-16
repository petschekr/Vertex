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
  float elevation;
};
SensorError sensorError = {0, 0, 0};
boolean calibrating = true;

struct Attitude {
  float pitch;
  float roll;
  float yaw;
};
Attitude attitude;
unsigned long lastAttitudeUpdate = 0;

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
  // Enable auto-ranging
  gyro.enableAutoRange(true);
  
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
  sensors_vec_t sensorData;
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
  lastAttitudeUpdate = millis();
  
  digitalWrite(armLED, HIGH);
}

void loop () {
  setThrottleAll(minThrottle);
  
  sensors_vec_t orientation = getOrientation();
  updateAttitude();
  //float altitude = getAltitude();
  Serial.print(F("Yaw: "));
  Serial.print(attitude.yaw);
  Serial.print(F(";\tRaw heading: "));
  Serial.println(orientation.heading);
  
  // When pitch is positive, decrease motors 1 and 4; increase 2 and 3
  
  // When roll is positive, decrease motors 3 and 4; increase 1 and 2
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
  attitude.pitch = 0.98 * (attitude.pitch + gyroData.x * elapsedSeconds) + 0.02 * orientation.pitch;
  attitude.roll = 0.98 * (attitude.roll + gyroData.y * elapsedSeconds) + 0.02 * orientation.roll;
  // Immediately correct when compass switches between -180 and 180 degrees
  if (abs(orientation.heading - attitude.yaw) > 100) {
    attitude.yaw = orientation.heading;
  }
  else {
    attitude.yaw = 0.98 * (attitude.yaw + gyroData.z * elapsedSeconds) + 0.02 * orientation.heading;
  }
  lastAttitudeUpdate = millis();
  delay(5);
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
  while (true) {
    digitalWrite(armLED, LOW);
    digitalWrite(readyLED, LOW);
    delay(250);
    digitalWrite(armLED, HIGH);
    digitalWrite(readyLED, HIGH);
    delay(250);
  }
}
