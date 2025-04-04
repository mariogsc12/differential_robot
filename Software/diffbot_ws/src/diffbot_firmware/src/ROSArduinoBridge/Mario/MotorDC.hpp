
#ifndef MOTORDC_HPP
#define MOTORDC_HPP

#include <Mario/Encoder.hpp>


#define MAX_PWM 255
#define MOTOR_DEAD_ZONE_THRESHOLD 50
#define MOTOR_RIGHT_ENABLE 5
#define MOTOR_RIGHT_IN1  6
#define MOTOR_RIGHT_IN2  9
#define MOTOR_LEFT_ENABLE 5
#define MOTOR_LEFT_IN1  6
#define MOTOR_LEFT_IN2  9
int saturate(int value, int min, int max);


class MotorDC
{
	public:
		MotorDC(const int pinEN, const int pinIN1, const int pinIN2,Encoder encoder);
		void initialize();
		void move();
		
	private:
        const int Pin_En;
        const int Pin_In1;
        const int Pin_In2;
		const int dead_zone_threshold;
        Encoder encoder;
	};
#endif