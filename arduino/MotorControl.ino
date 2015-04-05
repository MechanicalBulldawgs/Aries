#include <LiquidCrystal.h>

// Controls PWM controlled motors on the Adafruit 16 Channel PWM Arduino Shield.
// Takes a input range (-100 to 100) over serial, and is used to control the position(or direction and speed),
// of the servo/motor. 


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)





void setup() {
  Serial.begin(9600);
  Serial.println("Serial Monitor Servo Control");

  pwm.begin();
  
  pwm.setPWMFreq(60);  //Sets frequency to send to servo.
  
}

String readString;
int current_pos = 0; //Defaults these two values to 0
int target_pos = 0; 
int delay_time = 100;
void loop() { 
  
    //Reads data from the serial port
    while (Serial.available() > 0) {    // Loops in order to grab all data seen on serial line. 
      delay(10);
     if (Serial.available() > 0) {
      char data_in = Serial.read();     // Temp variable to hold single char value 
      readString += data_in;   }        // Concatenates string to form entire "message"
    }
    
    delay(10);
    
    // Keeps the target_pos at it's current value if data return empty string. 
    if (readString == "") {              
      target_pos = target_pos;
    }
    else {
      target_pos = readString.toInt(); 
    }
       
    //Implements Velocity Sooothing by incrementing the current_pos by 1 until it reaches the target position.
    // Decrements/Increments depending on whether current_pos is greater/less than target_pos respectively. 
    if (current_pos < target_pos) {
      
      int new_pwm_pos = map(current_pos + 1, -100, 100,SERVOMIN,SERVOMAX); //Maps input value(-100to100) to PWM ranges. 
      pwm.setPWM(0,0,new_pwm_pos);
      current_pos = current_pos + 1;  //Updates current_pos after control has been sent
    }
        
    if (current_pos > target_pos) { 
      
      int new_pwm_pos = map(current_pos - 1, -100, 100, SERVOMIN, SERVOMAX); //Maps input value(-100to100) to PWM ranges.
      pwm.setPWM(0,0,new_pwm_pos);
      current_pos = current_pos - 1;  //Updates current_pos after control has been sent
    }
        
    if (current_pos == target_pos) {
      
      int pwm_pos = map(current_pos, -100,100,SERVOMIN,SERVOMAX); //Maps input value(-100to100) to PWM ranges.
      pwm.setPWM(0,0,pwm_pos);
    }
            
          
    readString =""; //Resets readString to an empty string. 
  
    delay(delay_time); //Sets how long the Arduino waits before updating the ServoSpeed by 1.

}
