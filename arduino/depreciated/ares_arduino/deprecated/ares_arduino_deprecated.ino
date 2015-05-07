#include <Adafruit_L3GD20_U.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <math.h>

#define dt .025
#define twaitIdeal 25
#define tmath 11
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
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


//from Madgwick, for the MadgwickAHRSupdateIMU
#define betaDef		0.1f		// 2 * proportional gain
volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame


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
  
  if(!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
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
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Gyro")); Serial.println("");
  
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
  
  time1 = millis();
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
  
  /*Get gyro data*/
//  gyro.getEvent(&gyro_eventAvg);
//  for(int j = 0; j < 8; j++){
//   gyro.getEvent(&gyro_event);
//   
//   gyro_eventAvg.gyro.x += gyro_event.gyro.x;
//   gyro_eventAvg.gyro.y += gyro_event.gyro.y;
//   gyro_eventAvg.gyro.z += gyro_event.gyro.z;
//  }
  
  gyro_eventAvg.gyro.x = (gyro_eventAvg.gyro.x/10) - gyroXOffset;
  gyro_eventAvg.gyro.y = (gyro_eventAvg.gyro.y/10) - gyroYOffset;
  gyro_eventAvg.gyro.z = (gyro_eventAvg.gyro.z/10) - gyroZOffset;
  
  Serial.print("X: "); Serial.print(accel_eventAvg.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accel_eventAvg.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel_eventAvg.acceleration.z); Serial.println("  ");
  
  
  /* Display the results (speed is measured in rad/s) */
//  Serial.print("X: "); Serial.print(gyro_eventAvg.gyro.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(gyro_eventAvg.gyro.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(gyro_eventAvg.gyro.z); Serial.print("  ");
//  Serial.println("rad/s ");

//  xvel += (accel_event.acceleration.x + 0.037999) * dt;
//  yvel += (accel_event.acceleration.y - 0.5998899) * dt;
//  
//  xpos += xvel * dt;
//  ypos += yvel * dt;
//  Serial.print("xpos: ");Serial.print(xpos);
//  Serial.print(" ypos: ");Serial.print(ypos);
//  Serial.println(F(""));

  float fromQuaternion;
  MadgwickAHRSupdateIMU(gyro_eventAvg.gyro.x, gyro_eventAvg.gyro.y, gyro_eventAvg.gyro.z, accel_eventAvg.acceleration.x, accel_eventAvg.acceleration.y, accel_eventAvg.acceleration.z);
  
  pitch = pitchFromQuaternion() * 180/3.14159265359;
  roll = rollFromQuaternion() * 180/3.14159265359;
  heading = yawFromQuaternion() * 180/3.14159265359;
  
  calcVelocity(accel_eventAvg.acceleration.x, accel_eventAvg.acceleration.y);
  
  xpos += xvel * dt;
  ypos += yvel * dt;
  time2 = millis();
//  Serial.println(time2 - time1);
  
  Serial.print("roll: ");Serial.print(roll);
  Serial.print(" pitch: ");Serial.print(pitch);
  Serial.print(" heading: ");Serial.print(heading);
  Serial.println(F(""));
//  
//  Serial.print("xpos: ");Serial.print(xpos);
//  Serial.print(" ypos: ");Serial.print(ypos);
//  Serial.println(F(""));

  delay(twaitms);
}

float rollFromQuaternion(){
  return atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
}

float pitchFromQuaternion(){
 return asin(2*(q0*q2 - q3*q1)); 
}

float yawFromQuaternion(){
 return  atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}


void calcVelocity(float accelX, float accelY, float accelZ){
//convert to angled vector
 if((accelX < 0.15) and (accelX > -0.15))
   accelX = 0;
   
 if((accelY < 0.15) and (accelY > -0.15))
   accelY = 0;
 
 float r, theta, phi;
 r = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
 
 if(accelX != 0){
   theta = atan(accelY/accelX);
 }
 
 else{
   theta = 0;
 }

 //turn the angle by the heading, so all x and y are independent of the heading
 theta = theta - heading;
 
 //return to x and y vectors
 float x, y;
 x = r*cos(theta);
 y = r*sin(theta);
 
 xvel += x*dt;
 yvel += y*dt;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update from Madgwick

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f * dt);
	q1 += qDot2 * (1.0f * dt);
	q2 += qDot3 * (1.0f * dt);
	q3 += qDot4 * (1.0f * dt);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

