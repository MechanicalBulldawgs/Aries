#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>

//Timer Values
#define FREQUENCY 10 //in Hz, e.g. 10 = 10 Hz = 0.1s
bool timer1_flag = false;

//Motor Values
#define MOTOR_SCALE 0.902
#define MOTOR_OFFSET 370.0
#define MOTOR_MAX 600.0
#define MOTOR_MIN 150.0
float base_linear, base_angular, converted_output;

//Potentiometer Values
#define POT1_PIN A2
#define POT2_PIN A3
#define ADC_REF 5 //reference voltage of ADC (V)
#define GROVE_VCC 5 //VCC of the grove interface (V)
#define FULL_ANGLE 300 //full value of the rotary angle (degrees)
int pot1_angle;
int pot2_angle;

//IR Values
#define IR_PIN 6
bool ir_triggered = false;

#define MIN_ANGLE 55  //hopper shouldn't return past this point; send stop message
#define MAX_ANGLE 165 // hopper should stop here; send stop message

//IMU Values
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool accel_connected = false;
bool gyro_connected = false;


void setup(void)
{
  Serial.begin(9600);
  Serial.println(F("Ares Arduino")); Serial.println("");
  
  pwm.begin();
  pwm.setPWMFreq(60); //Sets frequency to send to servo. 
  initPotentiometers();
  initIR();
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
    readPotentiometers();
    readIR();
    printData();
    
  }
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

ISR(TIMER0_COMPA_vect) {
   timer1_flag = true;
}

void initIMU() {
  /**
  * Setup accelerometer and gyroscope, and set flags if they were initialized successfully
  */
  gyro.enableAutoRange(true);
  if(gyro.begin()) {gyro_connected = true;}
  if(accel.begin()) {accel_connected = true;}
}

void initPotentiometers() {
  /**
  * Set pins for potentiometers to inputs
  */
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
}

void initIR() {
  /**
  * Set pin for IR sensor to input mode
  */
  pinMode(IR_PIN, INPUT);
}

void readPotentiometers() {
  /**
  * Get value from potentiometers and convert to angles, then save the values to
  * pot1_angle and pot2_angle
  */
  int val1 = analogRead(POT1_PIN);
  int val2 = analogRead(POT2_PIN);
  float voltage1 = (float)val1*ADC_REF/1023;
  float voltage2 = (float)val2*ADC_REF/1023;
  pot1_angle = (voltage1*FULL_ANGLE)/GROVE_VCC;
  pot2_angle = (voltage2*FULL_ANGLE)/GROVE_VCC;
}

void readIR() {
  /**
  * Sets ir_triggered to 1 if the digital pin goes low, otherwise sets it to 0
  */
  ir_triggered = (digitalRead(IR_PIN) == LOW)? 1:0;
}

void printData() {
/**
* Prints out all sensor data as a single JSON string as described below:
{
    "data": {
        "imu": {
            "accel": {
                "x": 10,
                "y": 10,
                "z": 10,
                "pitch": 10,
                "roll": 10
            },
            "gyro": {
                "x": 10,
                "y": 10,
                "z": 10
            }
        },
        "potentiometers": {
            "pot1": 10,
            "pot2": 10
        },
        "ir": 10
    }
}
*/
  sensors_event_t event;
  sensors_vec_t   orientation;

  Serial.print("{\"data\": {");
  if (accel_connected || gyro_connected) {
    Serial.print("\"imu\": {");
  }
  if (accel_connected) {
    accel.getEvent(&event);
    Serial.print("\"accel\": {\"x\":");
    Serial.print(event.acceleration.x);
    Serial.print(", \"y\":");
    Serial.print(event.acceleration.y);
    Serial.print(", \"z\":");
    Serial.print(event.acceleration.z);
    if (dof.accelGetOrientation(&event, &orientation)) {
      Serial.print(", \"pitch\":");
      Serial.print(orientation.pitch);
      Serial.print(", \"roll\":");
      Serial.print(orientation.roll);
    }
    Serial.print("}");
    if (gyro_connected) {
      Serial.print(",");
    }
  }
  if (gyro_connected) {
    gyro.getEvent(&event);
    Serial.print("\"gyro\": {\"x\":");
    Serial.print(event.gyro.x);
    Serial.print(", \"y\":");
    Serial.print(event.gyro.y);
    Serial.print(", \"z\":");
    Serial.print(event.gyro.z);
    Serial.print("}");
  }
  if (accel_connected || gyro_connected) {
    Serial.print("},");
  }
  Serial.print("\"potentiometers\": {\"pot1\":");
  Serial.print(pot1_angle);
  Serial.print(", \"pot2\":");
  Serial.print(pot2_angle);
  Serial.print("}, \"ir\":");
  Serial.print(ir_triggered);
  Serial.print("}}\n");
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
      if (motor_num >= 0 && motor_num <= 9) {
        base_linear = value;
        converted_output = base_linear * MOTOR_SCALE + MOTOR_OFFSET;
        converted_data = limit_data(MOTOR_MIN, MOTOR_MAX, converted_output);
        pwm.setPWM(motor_num, 0, (int)converted_data);  
        Serial.print("X: "); 
        Serial.println(base_linear);
      }
    }
    else{
     Serial.flush(); 
    } 
  }
}
