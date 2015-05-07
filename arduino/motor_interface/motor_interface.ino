#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

//Timer Values
#define FREQUENCY 10 //in Hz, e.g. 10 = 10 Hz = 0.1s
bool timer1_flag = false;

//Motor Values
#define MOTOR_SCALE 0.902
#define MOTOR_OFFSET 370.0
#define MOTOR_MAX 600.0
#define MOTOR_MIN 150.0
float base_linear, base_angular, converted_output;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup(void)
{
  Serial.begin(9600);
  Serial.println(F("Ares Arduino Motor Interface")); Serial.println("");
  
  pwm.begin();
  pwm.setPWMFreq(60); //Sets frequency to send to servo. 
  init_timer1(FREQUENCY);

}

/**************************************************************************/
/*!
    @brief  Check IMU, Potentiometers, and IR sensor at a given frequency and send motor commands
*/
/**************************************************************************/
void loop(void)
{
  if (timer1_flag) {
    timer1_flag = false;

    update_motors();    
  }
//update_motors();
//delay(10);
}

int limit_data(int minData, int maxData, int data) {
  /**
  * Returns data, capped at the maximum and miminum values
  */
  if (data > maxData) {
    return maxData;
  } else if (data < minData){
    return minData;
  }
  return data;
}

void init_timer1(int frequency) {
  /**
  * Initialize timer1 at the given frequency
  * http://www.instructables.com/id/Arduino-Timer-Interrupts/
  */
  cli(); //disable interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = (16000000/(1024*frequency)) - 1;  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable interrupts
}

ISR(TIMER1_COMPA_vect) {
   timer1_flag = true;
}



void update_motors() {
  /**
  * Checks if a new motor command is available. If so, attempts to parse the command 
  * then converts the command to PWM values and publishes it to the motors.
  * If the command cannot be parsed, flushes the buffer.
  */
  int converted_data;
  
  if (Serial.available()) {
    char label = Serial.read();
    Serial.print("Label: "); Serial.print(label); 
    char separator = Serial.read();
    Serial.print("; Separator: "); Serial.println(separator); 
    if(separator == ':'){
      float value = Serial.parseFloat();
      Serial.print("; Value: "); Serial.println(value);

      uint8_t motor_num = label - '0';
      if (motor_num >= 0 && motor_num <= 1) {
        base_linear = value;
        converted_output = base_linear * MOTOR_SCALE + MOTOR_OFFSET;
        converted_data = limit_data(MOTOR_MIN, MOTOR_MAX, converted_output);
        pwm.setPWM(motor_num, 0, (int)converted_data);  
        Serial.print("X: "); 
        Serial.println(base_linear);
      }
      else if (motor_num >= 2 && motor_num <= 9) {
        base_linear = value;
        pwm.setPWM(motor_num, 0, (int)base_linear);  
        Serial.print("X: "); 
        Serial.println(base_linear);
        Serial.println(converted_data);
      }
    }
    else{
     Serial.flush(); 
    } 
  }
}
