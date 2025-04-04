#include <Arduino.h>
#include <Mario/MotorDC.hpp>



int saturate(int value, int min, int max)
{
	if(value < min) return min;
	else if(value > max) return max;
	else return value;
}

MMotorDC::MotorDC(const int pinEn, const int pinIn1, const int pinIn2,Encoder enc):
				Pin_En(pinEn),
				Pin_In1(pinIn1),
				Pin_In2(pinIn2),
				dead_zone_threshold(MOTOR_DEAD_ZONE_THRESHOLD)
				Encoder(std::move(enc))
{
}

void initialize()
{
	// initialize the gpio as a microcontroller output (ESP32 -> L298N)
	pinMode(pinEn,OUTPUT);  
	pinMode(pinIn1,OUTPUT);
	pinMode(pinIn2,OUTPUT);

}


void move(int pwm)
{
	if(pwm > dead_zone_threshold){
		digitalWrite(PinIn1,HIGH);
		digitalWrite(PinIn2,LOW);
		analogWrite(PinEn,abs(saturate(pwm,0,MAX_PWM)));
	}
	else if(pwm < dead_zone_threshold){
		digitalWrite(PinIn1,LOW);
		digitalWrite(PinIn2,HIGH);
		analogWrite(PinEn,abs(saturate(pwm,-MAX_PWM,0)));
	}
	else{
		digitalWrite(PinIn1,LOW);
		digitalWrite(PinIn2,LOW);
		analogWrite(PinEn,0);
	}
}
