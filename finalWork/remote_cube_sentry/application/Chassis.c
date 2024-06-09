//
// Created by Lumos on 2024/06/08.
//

/*include*/
#include "Chassis.h"
#include "user_lib.h"

chassis_t chassis = {
        .vx = 0,
        .vy = 0,
        .vw = 0,
};
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern uint8_t bRxBufferUart1[1]; //接收数据
extern int8_t wasdLR[6];
fp32 deltaSpeed = 0.005;
static fp32 rotate_ratio_f = ((Wheel_axlespacing + Wheel_spacing) / 2.0f - GIMBAL_OFFSET); //rad 0.4195左右
static fp32 rotate_ratio_b = ((Wheel_axlespacing + Wheel_spacing) / 2.0f + GIMBAL_OFFSET);//0.4195左右
static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO); //车轮转速比 2405左右
/*程序主体*/

void chassis_init()
{
    //底盘驱动电机速度环初始化和电机数据结构体获取
    for (int i = 0; i < 4; i++)
    {
        chassis.motor_chassis[i].motor_measure= motor_3508_measure + i;

        pid_init(&chassis.motor_chassis[i].speed_p,
                 CHASSIS_3508_PID_MAX_OUT,
                 CHASSIS_3508_PID_MAX_IOUT,
                 CHASSIS_3508_PID_KP,
                 CHASSIS_3508_PID_KI,
                 CHASSIS_3508_PID_KD);
    }

}

void chassis_speed_update()
{
    if(wasdLR[0])
        chassis.vx += deltaSpeed;
    if(wasdLR[2])
        chassis.vx -= deltaSpeed;
    if(!wasdLR[0] && !wasdLR[2])
        chassis.vx = 0;

    if(wasdLR[1] || wasdLR[3])
        chassis.vw = (wasdLR[1] - wasdLR[3]) * 5;
    if(!wasdLR[1] && !wasdLR[3])
        chassis.vw = 0;

//    if(wasdLR[1])
//        chassis.vy -= deltaSpeed;
//    if(wasdLR[3])
//        chassis.vy += deltaSpeed;
//    if(!wasdLR[1] && !wasdLR[3])
//        chassis.vy = 0;

//    if(wasdLR[4] || wasdLR[5])
//        chassis.vw = (wasdLR[4] - wasdLR[5]) * 5;
//    if(!wasdLR[4] && !wasdLR[5])
//        chassis.vw = 0;

    VAL_LIMIT(chassis.vx, -400, 400);
    VAL_LIMIT(chassis.vy, -400, 400);
    VAL_LIMIT(chassis.vw, -400, 400);
}

fp32 wheel_rpm[4] = {0};
void chassis_wheel_cal()
{
    fp32 vx, vy, vw;

    vx = chassis.vx;
    vy = chassis.vy;
    vw = chassis.vw;

    wheel_rpm[0] = (-vy - vx - vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = (-vy + vx - vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (vy + vx - vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (vy - vx - vw * rotate_ratio_b) * wheel_rpm_ratio;

    chassis.motor_chassis[RF].rpm_set=wheel_rpm[0];
    chassis.motor_chassis[LF].rpm_set=wheel_rpm[1];
    chassis.motor_chassis[LB].rpm_set=wheel_rpm[2];
    chassis.motor_chassis[RB].rpm_set=wheel_rpm[3];
}

void chassis_wheel_loop_cal()
{
    chassis.motor_chassis[RF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                              chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                              chassis.motor_chassis[RF].rpm_set);

    chassis.motor_chassis[LF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LF].speed_p,
                                                              chassis.motor_chassis[LF].motor_measure->speed_rpm,
                                                              chassis.motor_chassis[LF].rpm_set);

    chassis.motor_chassis[RB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RB].speed_p,
                                                              chassis.motor_chassis[RB].motor_measure->speed_rpm,
                                                              chassis.motor_chassis[RB].rpm_set);

    chassis.motor_chassis[LB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LB].speed_p,
                                                              chassis.motor_chassis[LB].motor_measure->speed_rpm,
                                                              chassis.motor_chassis[LB].rpm_set);

}

void chassis_can_send_back_mapping()
{
    int16_t *real_motor_give_current[4];

    real_motor_give_current[0] = &chassis.motor_chassis[LF].give_current;
    real_motor_give_current[1] = &chassis.motor_chassis[RF].give_current;
    real_motor_give_current[2] = &chassis.motor_chassis[RB].give_current;
    real_motor_give_current[3] = &chassis.motor_chassis[LB].give_current;

    CAN_cmd_motor(CAN_1,
                  CAN_MOTOR_0x200_ID,
                  *real_motor_give_current[0],
                  *real_motor_give_current[1],
                  *real_motor_give_current[2],
                  *real_motor_give_current[3]
    );

//    CAN_cmd_motor(CAN_1,
//                  CAN_MOTOR_0x200_ID,
//                  0,
//                  0,
//                  0,
//                  0
//    );
}

_Noreturn void chassis_task(void const *pvParameters) {

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
//    LoRa_T_V_Attach(1,1);
    chassis_init();

    //主任务循环
    while (1) {

        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误

        HAL_UART_Receive_IT(&huart1, bRxBufferUart1, 1);

        chassis_speed_update();

        chassis_wheel_cal();

        chassis_wheel_loop_cal();

        chassis_can_send_back_mapping();

        xTaskResumeAll();

        vTaskDelay(1);
    }

}
