#include <PID_v1.h>


double RSetpoint, RInput, ROutput;
double LSetpoint, LInput, LOutput;

//Specify the links and initial tuning parameters
PID Left_Motor(&LInput, &LOutput, &LSetpoint,2,5,1, DIRECT);
PID Right_Motor(&RInput, &ROutput, &RSetpoint,2,5,1, DIRECT); 

const float base_radius = .25;
void setup()
{

  //turn the PID on
  Left_Motor.SetMode(AUTOMATIC);
  Right_Motor.SetMode(AUTOMATIC);
}

void loop()
{
  //Retrieve Angular/Linear Velocities.
  double l_linear = 0;
  double r_linear = 0;
  double angular = 0;
  
  //Solve for Intended L/R Wheel Velocities Based of Angular/Linear Velocities.
  double l_velocity = l_linear - angular*base_radius;
  double r_velcotiy = r_linear + angular*base_radius; 
 
  //Compute Actual Velocities Based off of IMU Data
 
  //Convert All Velocities to Respective PWM Signal
    //This is based off motor characteristics, gearing, wheel sizes, etc. 
 
  //Set Inputs/Setpoints
 
  //Compute function(Uses the PID controller) 
  Left_Motor.Compute();
  Right_Motor.Compute();
  //Send the PWM Signal to each wheel
  
}
