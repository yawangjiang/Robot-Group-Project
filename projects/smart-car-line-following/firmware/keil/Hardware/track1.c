#include "stm32f10x.h"    
#include "Motor.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"

typedef struct
{
	float KP1;
	float KD;
	float GKD;
}Turn_PID;

Turn_PID turn_pid = {0.36,0.015,0.13};//(3)0.36,0.015,0.13 (2)0.36,0.015,0.10 (1)0.36,0.015,0.05
int16_t last_bias = 0;

static int16_t g_acc_x, g_acc_y, g_acc_z;
static int16_t g_gyro_x, g_gyro_y, g_gyro_z;

float Get_Gyro_Z(void)
{
    MPU6050_GetData(&g_acc_x, &g_acc_y, &g_acc_z, 
                    &g_gyro_x, &g_gyro_y, &g_gyro_z);
    
		float gyro_z_dps = g_gyro_z /16.4f;
    return gyro_z_dps;
}

void turn_loop(int16_t bias)
{
	float steer_err = turn_pid.KP1 * bias + turn_pid.KD * (bias - last_bias) - turn_pid.GKD * Get_Gyro_Z();
	
	last_bias = bias;
	float basic_speed = 27.0;
	
	int16_t target_speed_right,target_speed_left;
	
	if (steer_err > 80.0f)
		steer_err = 80.0f;
	else if (steer_err < -80.0f) 
		steer_err = -80.0f;
	
	if(steer_err >= 30)
	{
		target_speed_right = (int16_t)((basic_speed + steer_err) * 0.8) ;//0.8
		target_speed_left = (int16_t)((basic_speed - steer_err) * 0.8) ;//0.5
	}
	else if(steer_err <= -30){
		target_speed_right = (int16_t)((basic_speed + steer_err) * 0.8) ;//0.8
		target_speed_left = (int16_t)((basic_speed - steer_err) * 0.8) ;
	}
	else	
	{
		target_speed_right = (int16_t)((basic_speed + steer_err));
		target_speed_left = (int16_t)((basic_speed - steer_err));
	}

		LeftMotor_Speed(target_speed_left);
		RightMotor_Speed(target_speed_right);
}
