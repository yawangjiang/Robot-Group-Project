#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Smartcar.h"
#include "Track.h"
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "calculateBias.h"
#include "track1.h"

int main(void)
{
	Track_Init();
	Motor_Init();
	SmartCar_Init();
	MPU6050_Init();	
	
	while(1)
	{
		
		int16_t current_bias = Get_bias();
		turn_loop(current_bias);
		
	}	
	//RightMotor_Speed(-10);
	//LeftMotor_Speed(0);
}
