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
#include <Servo.h>

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
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; // 1014.1; //
float temperature;

sensors_event_t gyro_event;
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t bmp_event;
sensors_vec_t   orientation;
    
Servo servoPitch0;
Servo servoPitch1;
Servo servoRoll0;
Servo servoRoll1;   

int servoPitch0Trim = 3;
int servoPitch1Trim = 2;
int servoRoll0Trim  = 2;
int servoRoll1Trim  = 2;

void debugDump() {
  
  /* Display the results (speed is measured in rad/s) */
  Serial.print("X: "); Serial.print(gyro_event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyro_event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyro_event.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");
  
  Serial.print(F("Roll: "));
  Serial.print(orientation.roll);
  Serial.print(F("; "));
  Serial.print(F("Pitch: "));
  Serial.print(orientation.pitch);
  Serial.print(F("; "));
  Serial.print(F("Heading: "));
  Serial.print(orientation.heading);
  Serial.print(F("; "));

  Serial.print(F("Alt: "));
  Serial.print(bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature));
  Serial.print(F(" m; "));
  
  /* Display the temperature */
  Serial.print(F("Temp: "));
  Serial.print(temperature);
  Serial.print(F(" C"));
  Serial.println(F(""));

  Serial.print("servoPitch0.angle = "); Serial.println(servoPitch0.read());
  Serial.print("servoPitch1.angle = "); Serial.println(servoPitch1.read());
  Serial.print("servoRoll0.angle = "); Serial.println(servoRoll0.read());
  Serial.print("servoRoll1.angle = "); Serial.println(servoRoll1.read());
}

void initSensors()
{
  gyro.enableAutoRange(true);    
  gyro.begin();
  accel.begin();
  mag.begin();
  bmp.begin();
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

void setup(void)
{
  Serial.begin(9600);
  initSensors();
  initServos();
}

void loop(void)
{
  // Read all the sensors...
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  bmp.getEvent(&bmp_event);
  
  // Calculate some interesting things...
  dof.accelGetOrientation(&accel_event, &orientation);
  dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
  bmp.getTemperature(&temperature);

  // Adjust the servos...
  servoPitch0.write(map(orientation.pitch, SERVO_MIN_PITCH,SERVO_MAX_PITCH,1,180) + servoPitch0Trim);
  servoPitch1.write(map(orientation.pitch, SERVO_MAX_PITCH,SERVO_MIN_PITCH,1,180) + servoPitch1Trim);
  servoRoll0.write(map(orientation.roll, SERVO_MIN_ROLL,SERVO_MAX_ROLL,1,180) + servoRoll0Trim);
  servoRoll1.write(map(orientation.roll, SERVO_MAX_ROLL,SERVO_MIN_ROLL,1,180) + servoRoll1Trim);

  // dump stats once a second
  if ( millis() % 1000 < 100 ) {
    debugDump();
  }
}

