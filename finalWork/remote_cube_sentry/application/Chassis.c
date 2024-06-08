//
// Created by Lumos on 2024/06/08.
//

/*include*/
#include "Chassis.h"

chassis_t chassis;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern uint8_t bRxBufferUart1[1]; //接收数据
char message[] = "vx=10\r\nvy=10\r\nvw=10\r\n";
char test[] = "hello\r\n";
/*程序主体*/
_Noreturn void chassis_task(void const *pvParameters) {

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    LoRa_T_V_Attach(1,1);
    //主任务循环
    while (1) {

        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误

        HAL_UART_Receive_IT(&huart1, bRxBufferUart1, 1);

//        HAL_UART_Transmit_IT(&huart1, (uint8_t *)message, sizeof(message));

        xTaskResumeAll();

        vTaskDelay(1);
    }

}
