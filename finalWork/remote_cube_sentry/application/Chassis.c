//
// Created by Lumos on 2024/06/08.
//

/*include*/
#include "Chassis.h"

chassis_t chassis;
/*程序主体*/
_Noreturn void chassis_task(void const *pvParameters) {

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    LoRa_T_V_Attach(1,1);
    //主任务循环
    while (1) {

        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误

        xTaskResumeAll();

        vTaskDelay(1);
    }

}
