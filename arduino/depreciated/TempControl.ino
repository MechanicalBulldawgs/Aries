#include <Adafruit_L3GD20_U.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>

#define dt .15
#define twaitIdeal 205
#define tmath 202
#define twaitms twaitIdeal - tmath

#define accelXOffset -0.37
#define accelYOffset 0.09
#define accelZOffset 9.78

#define gyroXOffset -0.01
#define gyroYOffset 0.03
#define gyroZOffset 0.04

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float pitch, roll, heading, xpos, ypos, xvel, yvel;
double RSetpoint, RInput, ROutput;
double LSetpoint, LInput, LOutput;

//Specify the links and initial tuning parameters
PID Left_MotorPID(&LInput, &LOutput, &LSetpoint,0.25,0,0, DIRECT);
PID Right_MotorPID(&RInput, &ROutput, &RSetpoint,0.25,0,0, DIRECT); 
//Constants for robot motor control velocity equations
const float base_radius = .25;
const float wheel_radius = .1016;
const int R_motor = 0;
const int L_motor = 1;


float base_linear = 0;
float base_angular = 0;

int ROut = 370;
int LOut = 370;


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
  Serial.begin(9600);
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
  
  pwm.begin();
  pwm.setPWMFreq(60);      //Sets frequency to send to servo. 

  //turn the PID on
  Left_MotorPID.SetMode(AUTOMATIC);
  Right_MotorPID.SetMode(AUTOMATIC);
  
  
  Left_MotorPID.SetOutputLimits(-255,255);
  Right_MotorPID.SetOutputLimits(-225,255);
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
  
//  Serial.print("X: "); Serial.print(accel_eventAvg.acceleration.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(accel_eventAvg.acceleration.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(accel_eventAvg.acceleration.z); Serial.println("  ");
//  
//  Serial.print("X: "); Serial.print(gyro_eventAvg.gyro.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(gyro_eventAvg.gyro.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(gyro_eventAvg.gyro.z); Serial.println("  ");

  xvel += (accel_event.acceleration.x) * dt;
  yvel += (accel_event.acceleration.y) * dt;
  

  //Serial.println(time2 - time1);   //this can be removed once we know how long one loop iteration takes
  
  //Serial.print("xvel: ");Serial.print(xvel);
  //Serial.print(" yvel: ");Serial.print(yvel);
  //Serial.println(F(""));

  delay(twaitms);
  
  
  //Motor PID Code
  if (Serial.available() > 0) {
    
    delay(10);
    char label = Serial.read();
    Serial.print("Label: "); Serial.print(label); 
    char separator = Serial.read();
    Serial.print("; Separator: "); Serial.println(separator); 
    if(separator == ':'){
      float value = Serial.parseFloat();
      Serial.print("; Value: "); Serial.println(value);
      switch(label) {
       case '0': 
          pwm.setPWM(0,0,(int)value);
          break;   
       case '1': 
          pwm.setPWM(1,0,(int)value);
          break;
       case '2': 
          pwm.setPWM(2,0,400);
          Serial.println("case 2");
          break;
       case '3': 
          pwm.setPWM(3,0,(int)value);
          break;
       case '4': 
          pwm.setPWM(4,0,(int)value);
          break;
       case '5': 
          pwm.setPWM(5,0,(int)value);
          break;
       case '6': 
          pwm.setPWM(6,0,(int)value);
          break;
       case '7': 
          pwm.setPWM(7,0,(int)value);
          break; 
       case '8': 
          pwm.setPWM(8,0,(int)value);
          break;
       case '9': 
          pwm.setPWM(9,0,(int)value);
          break;
       case 'A': 
          pwm.setPWM(10,0,(int)value);
          break;
       case 'B': 
          pwm.setPWM(11,0,(int)value);
          break;
       case 'C': 
          pwm.setPWM(12,0,(int)value);
          break;
       case 'D': 
          pwm.setPWM(13,0,(int)value);
          break;
       case 'E': 
          pwm.setPWM(14,0,(int)value);
          break;
       case 'F': 
          pwm.setPWM(15,0,(int)value);
          break; 
       case 'X': 
          base_linear = value;
          Serial.print("X: "); Serial.println(base_linear);
          break;
       case 'Z': 
          base_angular = value;
          break;
       default: 
         break;
         
       
      }
    }
    else{
     Serial.flush(); 
    }
    
  }
    
    
  LOut = (base_linear*.902) + 370;
  ROut = (base_angular*.902) + 370;
 
  if (LOut > 600.0) {
     LOut = 600.0;
  }  
  else if (ROut > 600.0) {
      ROut = 600.0; 
  }
  else if (LOut < 150.0) {
   LOut = 150; 
  }
  else if (ROut < 150.0) {
   ROut = 150; 
  }
  
  Serial.print(ROut); Serial.print(" ");
  Serial.println(LOut);
  
  pwm.setPWM(R_motor,0,(int)ROut);
  pwm.setPWM(L_motor,0,(int)LOut);
  
  
  Serial.println();
  //delay(100);
    time2 = millis();                //this can be removed once we know how long one loop iteration takes
  //Serial.print(time2 - time1);
  
  


}
