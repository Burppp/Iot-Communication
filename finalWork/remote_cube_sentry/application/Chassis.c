//
// Created by Lumos on 2024/11/14.
//

/*include*/
#include "Chassis.h"
#include "user_lib.h"
#include "Lora.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern uint8_t bRxBufferUart1[1]; //接收数据
uint8_t frame_head[3] = {0x27, 0x66, 10};
uint8_t temp = 49;

fp32 data_origin = 0.1;
_Noreturn void chassis_task(void const *pvParameters) {

    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    TickType_t last_wake_time = xTaskGetTickCount();

    //LoRa_T_P_Attach(1,1);

    HAL_UART_Receive_IT(&huart1, bRxBufferUart1, 1);

    HAL_UART_Transmit(&huart1, (uint8_t *)frame_head, sizeof(frame_head), 0xff);

    //主任务循环
    while (1) {

        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误

        HAL_UART_Receive_IT(&huart1, bRxBufferUart1, 1);

        data_origin += 0.1;

//        HAL_UART_Transmit(&huart1, (uint8_t *)&data_origin, 4, 0xff);

        HAL_UART_Transmit(&huart1, (uint8_t *)frame_head, sizeof(frame_head), 0xff);

        HAL_UART_Transmit(&huart1, (uint8_t *)&temp, sizeof(temp), 0xff);

        xTaskResumeAll();

        //vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
        vTaskDelay(1000);
    }

}
