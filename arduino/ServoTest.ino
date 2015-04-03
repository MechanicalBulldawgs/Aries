// Controls PWM controlled motors on the Adafruit 16 Channel PWM Arduino Shield.
// Takes a input range (-100 to 100) over serial, and is used to control the position(or direction and speed),
// of the servo/motor. 


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


String readString;
//int servo_pos = 375;

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Monitor Servo Control");

  pwm.begin();
  
  pwm.setPWMFreq(60);  //Sets frequency to send to servo.
}


void loop() { 
    while (Serial.available() > 0) {    // Loops in order to grab all info seen on serial line. 
      delay(10);
     if (Serial.available() > 0) {
      char data_in = Serial.read();     // Temp variable to hold single char value 
      readString += data_in;   }        // Concatenates string to form entire "message"
     
     
    }
    
    delay(10);
    if (readString.length() >0) { // Only runs if something was received
      
      //Serial.println(readString.toInt());
      
      //int servo_pos = readString.toFloat(); // Converts string value to float
      //Serial.println(readString.toFloat());
      
      
      int servo_pos = map(readString.toInt(),-100,100,SERVOMIN,SERVOMAX); //Assigned as int since that is what the PWM object? accepts. 
      Serial.println(servo_pos);
      
      pwm.setPWM(0,0,servo_pos); //Sends the signal to the Servo.[Servo#,Signal High Start Time, Signal High End Time]
    }
    readString =""; //Resets readString to an empty string. 
    
   
  
    
    delay(100);

}
