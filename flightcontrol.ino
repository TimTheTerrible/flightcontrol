#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_SoftServo.h>  // SoftwareServo (works on non PWM pins)

// We demonstrate two servos!
#define SERVO1PIN  9  // Servo control line (orange) on Trinket Pin #0
#define SERVO2PIN 10  // Servo control line (orange) on Trinket Pin #1

#define POTPIN   1   // Potentiometer sweep (center) on Trinket Pin #2 (Analog 1)

Adafruit_SoftServo myServo1, myServo2;  //create TWO servo objects
   
/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
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

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();

  servo_setup();
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;
    
  if ( millis() % 1000 < 10 ) {
    gyro.getEvent(&gyro_event);
  
    /* Display the results (speed is measured in rad/s) */
    Serial.print("X: "); Serial.print(gyro_event.gyro.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(gyro_event.gyro.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(gyro_event.gyro.z); Serial.print("  ");
    Serial.println("rad/s ");
    
    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
      /* 'orientation' should have valid .roll and .pitch fields */
      Serial.print(F("Roll: "));
      Serial.print(orientation.roll);
      Serial.print(F("; "));
      Serial.print(F("Pitch: "));
      Serial.print(orientation.pitch);
      Serial.print(F("; "));
    }
    
    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
      /* 'orientation' should have valid .heading data now */
      Serial.print(F("Heading: "));
      Serial.print(orientation.heading);
      Serial.print(F("; "));
    }
  
    /* Calculate the altitude using the barometric pressure sensor */
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      /* Get ambient temperature in C */
      float temperature;
      bmp.getTemperature(&temperature);
      /* Convert atmospheric pressure, SLP and temp to altitude    */
      Serial.print(F("Alt: "));
      Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                          bmp_event.pressure,
                                          temperature)); 
      Serial.print(F(" m; "));
      /* Display the temperature */
      Serial.print(F("Temp: "));
      Serial.print(temperature);
      Serial.print(F(" C"));
    }
    
    Serial.println(F(""));
  }
  //delay(1000);
  
  servo_loop();
}

void servo_setup() {
  // Set up the interrupt that will refresh the servo for us automagically
  OCR0A = 0xAF;            // any number is OK
  TIMSK0 |= _BV(OCIE0A);    // Turn on the compare interrupt (below!)
  
  myServo1.attach(SERVO1PIN);   // Attach the servo to pin 0 on Trinket
  myServo1.write(90);           // Tell servo to go to position per quirk
  myServo2.attach(SERVO2PIN);   // Attach the servo to pin 1 on Trinket
  myServo2.write(90);           // Tell servo to go to position per quirk
  delay(15);                    // Wait 15ms for the servo to reach the position
}

void servo_loop()  {
  int potValue;  // variable to read potentiometer
  int servoPos;  // variable to convert voltage on pot to servo position
  potValue=analogRead(POTPIN);                // Read voltage on potentiometer
  servoPos = map(potValue, 0, 1023, 0, 179);  // scale it to use it with the servo (value between 0 and 180) 
  myServo1.write(servoPos);                    // tell servo to go to position
  myServo2.write(servoPos);                    // tell servo to go to position

  delay(15);                              // waits 15ms for the servo to reach the position
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
    myServo1.refresh();
    myServo2.refresh();
  }
}
