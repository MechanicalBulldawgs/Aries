/******************************************************************************/
/*macro definitions of Rotary angle sensor and LED pin*/
#define ROTARY_ANGLE_SENSOR A2
#define ADC_REF 5//reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
				 //board switches to 3V3, the ADC_REF should be 3.3
#define GROVE_VCC 5//VCC of the grove interface is normally 5v
#define FULL_ANGLE 300//full value of the rotary angle is 300 degrees

#define MIN_ANGLE 55  //hopper shouldn't return past this point; send stop message
#define MAX_ANGLE 165 // hopper should stop here; send stop message


void setup() 
{
	Serial.begin(9600);
	pinsInit();
}

void loop() 
{
	int degrees;
	degrees = getDegree();
	//Serial.println("The angle between the mark and the starting position:");
        String str = String(degrees);
	Serial.println("@POT_HOPPER "+str);
	
//        if (degrees >= MAX_ANGLE) {
//         Serial.println("Hopper has reached max angle, stop motor(ROSify this)");
//        } 
//        
//        if (degrees <= MIN_ANGLE) {
//          Serial.println("Hopper has reached min angle, stop motor(ROSify this)");
//        }
        
	delay(500);
}
void pinsInit()
{
	pinMode(ROTARY_ANGLE_SENSOR, INPUT);
}


/************************************************************************/
/*Function: Get the angle between the mark and the starting position	*/
/*Parameter:-void														*/
/*Return:	-int,the range of degrees is 0~300							*/
int getDegree()
{
	int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
	float voltage;
	voltage = (float)sensor_value*ADC_REF/1023;
	float degrees = (voltage*FULL_ANGLE)/GROVE_VCC;
	return degrees;
}
