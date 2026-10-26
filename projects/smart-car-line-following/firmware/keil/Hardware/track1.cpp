#include "stm32f10x.h"    
#include "Motor.h"
typedef struct
{
	float KP1;
	float KD;
	float GKD;
}Turn_PID;
Turn_PID turn_pid = {0.5,0.2,0};

int16_t last_bias = 0;

void turn_loop(int16_t bias)
{
	last_bias = bias;
	int16_t steer_err = turn_pid.KP1 * bias + turn_pid.KD * (bias - last_bias);
	int8_t basic_speed;
	int8_t target_speed_right,target_speed_left;
	if(steer_err >= 0)
	{
		target_speed_right = basic_speed - steer_err;//0.8
		target_speed_left = basic_speed + steer_err;//0.5
	}
	else	
	{
		target_speed_right = basic_speed - steer_err;
		target_speed_left = basic_speed + steer_err;
	}

		LeftMotor_Speed(target_speed_left);
		RightMotor_Speed(target_speed_right);
}
