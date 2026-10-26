#include "stm32f10x.h"                  // Device header
#include "Track.h"
#include "Motor.h"
#include "Smartcar.h"
#include "Delay.h"
void Track_Init(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5 |GPIO_Pin_15| GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}
void  Track_GetNum(void)
{

    if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)==1))
	{
		Motor_Stop();
	}	
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==0))
	{
		Motor_Contrarotate();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Forward();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Forward();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Forward();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==0))
	{
		Motor_Turn_Left();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Turn_Left();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==0))
	{
		Motor_Turn_Left();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Turn_Left();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_DeviationToRight();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_DeviationToRight();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_DeviationToRight();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Inclined();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Inclined();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Inclined();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Turn_Right();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Turn_Right();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Turn_Right();
	}
	if
 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
	{
		Motor_Turn_Right();
	}
//	if
// ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
//	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
//	{
//	Motor_DeviationToRight();
//	}
//	if
// ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)&&
//	(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3 )==0)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4 )==1)&&
//	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5 )==1))
//	{
//		Motor_Inclined();
//	}
}


