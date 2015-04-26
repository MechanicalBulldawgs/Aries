#include <Adafruit_L3GD20_U.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <math.h>

#define dt .025
#define twaitIdeal 25
#define tmath 21
#define twaitms twaitIdeal - tmath

#define accelXOffset -0.35
#define accelYOffset 0.02
#define accelZOffset 0.00

#define gyroXOffset -0.01
#define gyroYOffset 0.04
#define gyroZOffset 0.04

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);



float pitch, roll, heading, xpos, ypos, xvel, yvel;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  /* Initialise the sensor */
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
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Ares Arduino")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
  pitch = 0;
  roll = 0;
  heading = 0;
  xpos = 0;
  ypos = 0;
  xvel = 0;
  yvel = 0;
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  sensors_event_t accel_event, accel_eventAvg;
  sensors_event_t gyro_event, gyro_eventAvg;
  
  float time1, time2;
  time1 = millis();  //This can be removed once we know the time for a loop to execute
  
  /* Get the raw accelerometer data from an average of measurements */
  accel.getEvent(&accel_eventAvg);
  gyro.getEvent(&gyro_eventAvg);

  for(int i = 0; i < 8; i++){
    accel.getEvent(&accel_event);
    
    accel_eventAvg.acceleration.x += accel_event.acceleration.x;
    accel_eventAvg.acceleration.y += accel_event.acceleration.y;
    accel_eventAvg.acceleration.z += accel_event.acceleration.z;
    
    gyro.getEvent(&gyro_event);
    
    gyro_eventAvg.gyro.x += gyro_event.gyro.x;
    gyro_eventAvg.gyro.y += gyro_event.gyro.y;
    gyro_eventAvg.gyro.z += gyro_event.gyro.z;
    
  }
  
  accel_eventAvg.acceleration.x = (accel_eventAvg.acceleration.x/10) - accelXOffset;
  accel_eventAvg.acceleration.y = (accel_eventAvg.acceleration.y/10) - accelYOffset;
  accel_eventAvg.acceleration.z = (accel_eventAvg.acceleration.z/10) - accelZOffset;
  
  gyro_eventAvg.gyro.x = (gyro_eventAvg.gyro.x/10) - gyroXOffset;
  gyro_eventAvg.gyro.y = (gyro_eventAvg.gyro.y/10) - gyroYOffset;
  gyro_eventAvg.gyro.z = (gyro_eventAvg.gyro.z/10) - gyroZOffset;
  
  Serial.print("X: "); Serial.print(accel_eventAvg.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accel_eventAvg.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel_eventAvg.acceleration.z); Serial.println("  ");

  xvel += (accel_event.acceleration.x) * dt;
  yvel += (accel_event.acceleration.y) * dt;
  
  time2 = millis();                //this can be removed once we know how long one loop iteration takes
  Serial.println(time2 - time1);   //this can be removed once we know how long one loop iteration takes
  
  Serial.print("xvel: ");Serial.print(xvel);
  Serial.print(" yvel: ");Serial.print(yvel);
  Serial.println(F(""));

  delay(twaitms);
}
