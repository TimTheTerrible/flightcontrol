#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_SoftServo.h>

#define SERVO_PITCH0_PIN  9
#define SERVO_PITCH1_PIN 10
#define SERVO_ROLL0_PIN  11
#define SERVO_ROLL1_PIN   6

Adafruit_SoftServo servoPitch0;
Adafruit_SoftServo servoPitch1;
Adafruit_SoftServo servoRoll0;
Adafruit_SoftServo servoRoll1;
   
/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void initSensors()
{
  /* Enable gyro auto-ranging */
  gyro.enableAutoRange(true);
    
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

void initServos() {
  // Set up the interrupt that will refresh the servo for us automagically
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  
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
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();

  initServos();
}

void loop(void)
{
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;
    
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  bmp.getEvent(&bmp_event);
  
  dof.accelGetOrientation(&accel_event, &orientation);
  dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
  float temperature;
  bmp.getTemperature(&temperature);

  long pitch0 = map(orientation.pitch, -45,45,0,179);
  long pitch1 = map(orientation.pitch, 45,-45,0,179);
  long roll0 = map(orientation.roll, -45,45,0,179);
  long roll1 = map(orientation.roll, 45,-45,0,179);

  servoPitch0.write(pitch0);
  servoPitch1.write(pitch1);
  servoRoll0.write(roll0);
  servoRoll1.write(roll1);

  if ( millis() % 1000 < 100 ) {
  
    Serial.print("pitch0 = "); Serial.println(pitch0);
    Serial.print("pitch1 = "); Serial.println(pitch1);
    Serial.print("roll0 = "); Serial.println(roll0);
    Serial.print("roll1 = "); Serial.println(roll1);
    
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
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature)); 
    Serial.print(F(" m; "));
    /* Display the temperature */
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.print(F(" C"));
    Serial.println(F(""));
  }
}

// We'll take advantage of the built in millis() timer that goes off
// to keep track of time, and refresh the servo every 20 milliseconds
// The SIGNAL(TIMER0_COMPA_vect) function is the interrupt that will be
// Called by the microcontroller every 2 milliseconds
volatile uint8_t counter = 0;
SIGNAL(TIMER0_COMPA_vect) {
  // this gets called every 2 milliseconds
  counter += 2;
  // every 20 milliseconds, refresh the servos!
  if (counter >= 20) {
    counter = 0;

    servoPitch0.refresh();
    servoPitch0.refresh();
    servoRoll0.refresh();
    servoRoll1.refresh();
  }
}
