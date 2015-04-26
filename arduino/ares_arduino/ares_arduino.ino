#include <Adafruit_L3GD20_U.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>

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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float pitch, roll, heading, xpos, ypos, xvel, yvel;
double RSetpoint, RInput, ROutput;
double LSetpoint, LInput, LOutput;

//Specify the links and initial tuning parameters
PID Left_MotorPID(&LInput, &LOutput, &LSetpoint,2,5,1, DIRECT);
PID Right_MotorPID(&RInput, &ROutput, &RSetpoint,2,5,1, DIRECT); 
//Constants for robot motor control velocity equations
const float base_radius = .25;
const float wheel_radius = .1016;
const int R_motor = 0;
const int L_motor = 1;


float base_linear = 0;
float base_angular = 0;


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
  
  pwm.begin();
  pwm.setPWMFreq(60);      //Sets frequency to send to servo. 

  //turn the PID on
  Left_MotorPID.SetMode(AUTOMATIC);
  Right_MotorPID.SetMode(AUTOMATIC);
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
  
  //Serial.print("X: "); Serial.print(accel_eventAvg.acceleration.x); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(accel_eventAvg.acceleration.y); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(accel_eventAvg.acceleration.z); Serial.println("  ");

  xvel += (accel_event.acceleration.x) * dt;
  yvel += (accel_event.acceleration.y) * dt;
  
  time2 = millis();                //this can be removed once we know how long one loop iteration takes
  //Serial.println(time2 - time1);   //this can be removed once we know how long one loop iteration takes
  
  //Serial.print("xvel: ");Serial.print(xvel);
  //Serial.print(" yvel: ");Serial.print(yvel);
  //Serial.println(F(""));

  delay(twaitms);
  
  
  //Motor PID Code
  if (Serial.available() > 0) {
    
    delay(10);
    char label = Serial.read();
    char separator = Serial.read();
    if(separator == ':'){
      int value = Serial.parseInt();
      switch(label) {
       case '0': 
          pwm.setPWM(0,0,value);
          break;   
       case '1': 
          pwm.setPWM(1,0,value);
          break;
       case '2': 
          pwm.setPWM(2,0,value);
          break;
       case '3': 
          pwm.setPWM(3,0,value);
          break;
       case '4': 
          pwm.setPWM(4,0,value);
          break;
       case '5': 
          pwm.setPWM(5,0,value);
          break;
       case '6': 
          pwm.setPWM(6,0,value);
          break;
       case '7': 
          pwm.setPWM(7,0,value);
          break; 
       case '8': 
          pwm.setPWM(8,0,value);
          break;
       case '9': 
          pwm.setPWM(9,0,value);
          break;
       case 'A': 
          pwm.setPWM(10,0,value);
          break;
       case 'B': 
          pwm.setPWM(11,0,value);
          break;
       case 'C': 
          pwm.setPWM(12,0,value);
          break;
       case 'D': 
          pwm.setPWM(13,0,value);
          break;
       case 'E': 
          pwm.setPWM(14,0,value);
          break;
       case 'F': 
          pwm.setPWM(15,0,value);
          break; 
       case 'X': 
          base_linear = value;
          break;
       case 'Z': 
          base_angular = value;
          break;
         
       
      }
    }
    else{
     Serial.flush(); 
    }
    
    
  
  }  

  
  
  //Solve for Intended L/R Wheel Velocities Based of intended Angular/Linear Velocities.
  LSetpoint = base_linear - base_angular*base_radius;  //This is the desired velocity of the left wheel.
  RSetpoint = base_linear + base_angular*base_radius;  //This is the desired velocity of the right wheel. 
 
  //Compute Actual Velocities Based off of IMU Data
  //float xvel = 0;    //xvel = IMU measured base linear velocity
  float ang_vel = gyro_eventAvg.gyro.z; //ang_vel = IMU measured base angular velocity
  
  //Solve for Measured L/R Wheel Velocities based off measured Angular/Linear Velcoities. 
  LInput = xvel - ang_vel*base_radius;
  RInput = xvel + ang_vel*base_radius;  
 
  //Compute function(Uses the PID controller) 
  Left_MotorPID.Compute();
  Right_MotorPID.Compute();
  
  //Convert All Velocities to Respective PWM Signal
    //This is based off motor characteristics, gearing, wheel sizes, etc.
    
    //Equation relating wheel velcoity to PWM value
    int R_PWM = (280*ROutput) + 375;
    int L_PWM = (280*LOutput) + 375; 
    
  
    
  //Send the PWM Signal to each wheel
  pwm.setPWM(R_motor,0,R_PWM);
  pwm.setPWM(L_motor,0,L_PWM);
  
  
  
}
