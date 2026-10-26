#include "stm32f10x.h"  

static int32_t out_cnt;
int16_t Get_bias(void)
{
    uint8_t sensor_state[8];
    int16_t current_bias = 0;
    static int16_t last_bias = 0;
    
    const int16_t weights[8] = {-85, -50, -20, -10, 10, 20, 50, 85};  
    
    sensor_state[0] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
    sensor_state[1] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
    sensor_state[2] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
    sensor_state[3] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
    sensor_state[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
    sensor_state[5] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
    sensor_state[6] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
    sensor_state[7] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
    
    if(!sensor_state[0] && !sensor_state[1] && !sensor_state[2] && !sensor_state[3] && 
       !sensor_state[4] && !sensor_state[5] && !sensor_state[6] && !sensor_state[7]) {
        current_bias = 0;
    }
    
    else {
        int8_t active_sensors = 0;
        current_bias = 0;
        
        for(int i = 0; i < 8; i++) {
            if(sensor_state[i]) {
                current_bias += weights[i];
                active_sensors ++;
            }
        }
				//work
				if ((8-active_sensors) == 1 && !sensor_state[0])
					current_bias += 60;
				
				else if ((8-active_sensors) == 1 && !sensor_state[7])
					current_bias += -60;
        
        if( (8 - active_sensors) > 0) {
            out_cnt = 0;
        } 
        
        else {
            current_bias = last_bias;
            out_cnt++;
        }
        
        if(out_cnt > 10)
        {
			
        }
        
        if(current_bias > 1000) current_bias = 1000;
        if(current_bias < -1000) current_bias = -1000;
    }
    
    current_bias = 0.1*last_bias + 0.9* current_bias;
    last_bias = current_bias;
		
    return current_bias;
}
