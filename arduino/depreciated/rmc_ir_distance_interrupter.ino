#include <avr/interrupt.h>

// Timer interrupt flag
volatile boolean timer_flag = false;

//Pins
const int IR_DIST_INTER = 6;

void setup()  {
  Serial.begin(9600);
  pinMode(IR_DIST_INTER, INPUT);
  
  cli(); // stop interrupts
  //set timer1 interrupt at 10Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 10hz increments
  OCR1A = 1561;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();
}

// Function to service timer1 overflow interrupt.
ISR(TIMER1_COMPA_vect) {
  timer_flag = true;
}

void loop()  {
  
  boolean timer_flag_copy;
  // Protect the shared variable
  noInterrupts();
  timer_flag_copy = timer_flag;    // Grab the shared timer_flag variable
  interrupts();  
  
  if (timer_flag_copy) {
    //protect shared variable
    noInterrupts();
    timer_flag = false;  // reset shared timer_flag
    interrupts(); 
    
    /////////////////////////////////
    // IR distance interrupter 
    /////////////////////////////////
    Serial.print("@IR_Inter ");
    if(digitalRead(IR_DIST_INTER) == LOW)  {
      // Something is detected
      Serial.print("1");
    } else {
      // nothing is detected
      Serial.print("0");
    }
    Serial.print("\n");
     
  }
    
}
