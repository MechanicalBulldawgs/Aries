#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <PID_v1.h>


double RSetpoint, RInput, ROutput;
double LSetpoint, LInput, LOutput;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Specify the links and initial tuning parameters
PID Left_MotorPID(&LInput, &LOutput, &LSetpoint,2,5,1, DIRECT);
PID Right_MotorPID(&RInput, &ROutput, &RSetpoint,2,5,1, DIRECT); 

const float base_radius = .25;
const float wheel_radius = .1016;
const int R_motor = 0;
const int L_motor = 1;
void setup()
{
  pwm.begin();
  pwm.setPWMFreq(60);      //Sets frequency to send to servo. 

  //turn the PID on
  Left_MotorPID.SetMode(AUTOMATIC);
  Right_MotorPID.SetMode(AUTOMATIC);
  
}

void loop()
{
  //Retrieve Angular/Linear Velocities.
  float base_linear = 0;
  float angular = 0;
  
  //Solve for Intended L/R Wheel Velocities Based of intended Angular/Linear Velocities.
  LSetpoint = base_linear - angular*base_radius;  //This is the desired velocity of the left wheel.
  RSetpoint = base_linear + angular*base_radius;  //This is the desired velocity of the right wheel. 
 
  //Compute Actual Velocities Based off of IMU Data
  float xvel = 0;    //xvel = IMU measured base linear velocity
  float ang_vel = 0; //ang_vel = IMU measured base angular velocity
  
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
