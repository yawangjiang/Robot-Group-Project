#include "stm32f10x.h"                  // Device header
#include "Motor.h"  

void SmartCar_Init(void)
{
Motor_Init();
}

//void Motor_Backward(void)
//{
//    LeftMotor_Speed(-50);
//	RightMotor_Speed(-50);
//}
void Motor_Stop(void)
{
  LeftMotor_Speed(-8);
	RightMotor_Speed(-8);
}
//void Motor_Clockwise_Rotation(void)
//{
//    LeftMotor_Speed(50);
//	RightMotor_Speed(-50);
//}
void Motor_Contrarotate(void)
{
    LeftMotor_Speed(8);
	RightMotor_Speed(8);
}
void Motor_Turn_Right(void)
{
    LeftMotor_Speed(-8);
	RightMotor_Speed(20);
}

void Motor_DeviationToRight(void)
{
  LeftMotor_Speed(15);
	RightMotor_Speed(-10);
}
void Motor_Forward(void)
{
  LeftMotor_Speed(25);
	RightMotor_Speed(25);
}
void Motor_Inclined(void)
{
		LeftMotor_Speed(-10);
		RightMotor_Speed(15);
}
void Motor_Turn_Left(void)
{
    LeftMotor_Speed(20);
		RightMotor_Speed(-8);
}	




