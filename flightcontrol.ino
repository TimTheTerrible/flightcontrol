/*
 * flightcontrol
 *
 * 2015-10-02 - Tim Currie
 *
 * An Arduino-based flight control module for large model rockets.
 * Uses the Adafruit 10DOF combined sensor breakout driving four
 * servo-mounted winglets to maintain roll-free vertical flight.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_TMP006.h>
#include <Servo.h>
#include "debugprint.h"

#define SERVO_PITCH0_PIN      3
#define SERVO_PITCH1_PIN      4
#define SERVO_ROLL0_PIN       5
#define SERVO_ROLL1_PIN       6

#define SERVO_MIN_PITCH    -120
#define SERVO_MAX_PITCH     120
#define SERVO_MIN_ROLL     -120
#define SERVO_MAX_ROLL      120

#define SERVO_MIN_ANGLE       0
#define SERVO_MAX_ANGLE     180

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof    = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel  = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag    = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp    = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro   = Adafruit_L3GD20_Unified(20);
Adafruit_TMP006               irtemp = Adafruit_TMP006();

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; // 1014.1; //
float temperature;

sensors_event_t gyro_event;
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t bmp_event;
sensors_vec_t   orientation;
double IRTempObject;
double IRTempDie;
long nextRead;
    
Servo servoPitch0;
Servo servoPitch1;
Servo servoRoll0;
Servo servoRoll1;   

int servoPitch0Trim = 3;
int servoPitch1Trim = 2;
int servoRoll0Trim  = 2;
int servoRoll1Trim  = 2;

IntervalTimer debugTimer;
boolean doDebugDump = false;
long sampleCount = 0;
const int debugLEDPin = 13;

float servoGainPitch = 3;
float servoGainRoll = 3;

void debugDump() {
  digitalWrite(debugLEDPin, HIGH);
  
  long m = millis();
  debugprint(DEBUG_INFO, "\n*** Status at %3.3d:%3.3d", m / 1000, m % 1000);
  
  debugprint(DEBUG_INFO, "Gyro:\nX: %6.2f  Y: %6.2f  Z: %6.2f  rad/s",
    gyro_event.gyro.x, gyro_event.gyro.y, gyro_event.gyro.z);
  
  debugprint(DEBUG_INFO, "Accelerometer:\nRoll: %6.2f; Pitch: %6.2f; Heading: %6.2f",
    orientation.roll, orientation.pitch, orientation.heading);

  debugprint(DEBUG_INFO, "Barometer:\nAlt: %7.3f m; Temp: %4.1f C",
    bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature), temperature);
  
  debugprint(DEBUG_INFO, "Servos:\nP0: %d  P1: %d  R0: %d  R1: %d",
    servoPitch0.read(), servoPitch1.read(), servoRoll0.read(), servoRoll1.read());

  debugprint(DEBUG_INFO, "IR Temp:\nObject: %6.2fC  Die: %6.2f ", IRTempObject, IRTempDie);
  
  debugprint(DEBUG_INFO, "Based on %d samples", sampleCount);
  
  sampleCount = 0;
  doDebugDump = false;
  digitalWrite(debugLEDPin, LOW);
}

void initSensors()
{  
  gyro.enableAutoRange(true);    
  gyro.begin();
  accel.begin();
  mag.begin();
  bmp.begin();
  irtemp.begin();
}

void initServos() {  
  servoPitch0.attach(SERVO_PITCH0_PIN);
  servoPitch0.write(90);
  servoPitch1.attach(SERVO_PITCH1_PIN);
  servoPitch1.write(90);
  servoRoll0.attach(SERVO_ROLL0_PIN);
  servoRoll0.write(90);
  servoRoll1.attach(SERVO_ROLL1_PIN);
  servoRoll1.write(90);
}

void blinkLED(int count) {
  while ( count-- > 0 ) {
    digitalWrite(debugLEDPin, HIGH);
    delay(100);
    digitalWrite(debugLEDPin, LOW);
    delay(100);
  }
}

void setup(void)
{
  Serial.begin(115200);
  delay(500);
  debugprint(DEBUG_INFO, "Starting up!");
  pinMode(debugLEDPin, OUTPUT);
  digitalWrite(debugLEDPin, LOW);

  blinkLED(3);

  initSensors();
  
  initServos();

  debugTimer.begin(callDebug, 1000000);
  
  nextRead = millis() + 4000;
  
  blinkLED(3);
}

void callDebug() {
  doDebugDump = true;
}

void readSensors() {
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  bmp.getEvent(&bmp_event);
  
  // Calculate pitch and roll...
  dof.accelGetOrientation(&accel_event, &orientation);

  // Calculate heading...
  dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);

  // Get the ambient temperature...
  bmp.getTemperature(&temperature);

  // Read the IR engine temp sensor
  if ( millis() > nextRead ) {  // the sensor can only be read once every four seconds...
    IRTempObject = irtemp.readObjTempC();
    IRTempDie = irtemp.readDieTempC();
    Serial.println(IRTempObject);
    Serial.println(IRTempDie);
    nextRead = millis() + 4000;
  }

  sampleCount += 1;
}

void adjustServos() {
  servoPitch0.write(map(orientation.pitch * servoGainPitch, SERVO_MIN_PITCH,SERVO_MAX_PITCH,1,180) + servoPitch0Trim);
  servoPitch1.write(map(orientation.pitch * servoGainPitch, SERVO_MAX_PITCH,SERVO_MIN_PITCH,1,180) + servoPitch1Trim);
  servoRoll0.write(map(orientation.roll * servoGainRoll, SERVO_MIN_ROLL,SERVO_MAX_ROLL,1,180) + servoRoll0Trim);
  servoRoll1.write(map(orientation.roll * servoGainRoll, SERVO_MAX_ROLL,SERVO_MIN_ROLL,1,180) + servoRoll1Trim);
}
  
void loop(void)
{
  readSensors();

  adjustServos();
  
  if ( doDebugDump ) {
    debugDump();
  }
}

