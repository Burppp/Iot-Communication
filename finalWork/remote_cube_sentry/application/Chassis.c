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
        .relax = 1
};
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern uint8_t bRxBufferUart1[1]; //接收数据
extern int8_t wasdLR[6];
fp32 deltaSpeed = 2.5;
static fp32 rotate_ratio_f = ((Wheel_axlespacing + Wheel_spacing) / 2.0f - GIMBAL_OFFSET); //rad 0.4195左右
static fp32 rotate_ratio_b = ((Wheel_axlespacing + Wheel_spacing) / 2.0f + GIMBAL_OFFSET);//0.4195左右
static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO); //车轮转速比 2405左右

float speed_set;
float aver_speed;
float turn_speed_set;
float ins_angle[3];
motor_t motorL;
motor_t motorR;
pid_t standstill_pid;
float speed_out_r;
float target_roll = -0.8f;//-1.9
extern fp32 INS_angle[3];
/*程序主体*/

void chassis_init()
{
    //底盘驱动电机速度环初始化和电机数据结构体获取
    for (int i = 0; i < 4; i++)
    {
        chassis.motor_chassis[i].motor_measure= motor_2006_measure + i;
        pid_init(&chassis.motor_chassis[i].speed_p,
                 CHASSIS_2006_PID_MAX_OUT,
                 CHASSIS_2006_PID_MAX_IOUT,
                 CHASSIS_2006_PID_KP,
                 CHASSIS_2006_PID_KI,
                 CHASSIS_2006_PID_KD);
    }
    chassis.vx = 0;
    chassis.vw = 0;

    pid_init(&standstill_pid, 1000, 1000, 38, 0.0, 750);
    pid_init(&motorL.pid, 500, 200, 24, 0.00f, 90);
    pid_init(&motorR.pid, 500, 200, 24, 0.00f, 90);
    first_Kalman_Create(&motorR.kalman, 1, 1);
    first_Kalman_Create(&motorL.kalman, 1, 1);
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
        chassis.vw = (wasdLR[1] - wasdLR[3]) * 2000;
    if(!wasdLR[1] && !wasdLR[3])
        chassis.vw = 0;

    if(wasdLR[4])
    {
        chassis.relax = 1;
        //standstill_pid.iout = 0;
    }
    else
        chassis.relax = 0;
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

//    VAL_LIMIT(chassis.vx, -400, 400);
//    VAL_LIMIT(chassis.vy, -400, 400);
//    VAL_LIMIT(chassis.vw, -400, 400);
}

void change_current_to_pwm(motor_t *motor)
{

    motor->pwm1=(uint16_t)(1000+motor->give_current);
    motor->pwm2=(uint16_t)(1000-motor->give_current);
    if(motor->pwm1>2000)
    {
        motor->pwm1=2000;
    }
    if(motor->pwm2>2000)
    {
        motor->pwm2=2000;
    }
    if(motor->pwm1<0)
    {
        motor->pwm1=0;
    }
    if(motor->pwm2<0)
    {
        motor->pwm2=0;
    }
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

        if(chassis.relax == 1)
        {
            speed_set = (float) (chassis.vx) * 0.03f;
            turn_speed_set = (float) (chassis.vw) * 0.08f;
            ins_angle[0] = INS_angle[0] * MOTOR_RAD_TO_ANGLE;
            ins_angle[1] = INS_angle[1] * MOTOR_RAD_TO_ANGLE;
            ins_angle[2] = INS_angle[2] * MOTOR_RAD_TO_ANGLE;

            first_Kalman_Filter(&motorL.kalman, motorL.speed);
            first_Kalman_Filter(&motorR.kalman, motorR.speed);
            float angle_loop_out = pid_calc(&standstill_pid, ins_angle[2], target_roll);
            aver_speed = (-motorL.kalman.X_now + motorR.kalman.X_now) / 2;
            speed_out_r = pid_calc(&motorR.pid, aver_speed, speed_set);
            motorR.give_current = angle_loop_out + speed_out_r + turn_speed_set;
            motorL.give_current = -angle_loop_out - speed_out_r + turn_speed_set;
            change_current_to_pwm(&motorL);
            change_current_to_pwm(&motorR);
            can_send_motor_lg(&motorL, &motorR);
        }
        else if(chassis.relax == 0)
        {
            motorR.give_current = 0;
            motorL.give_current = 0;
            change_current_to_pwm(&motorL);
            change_current_to_pwm(&motorR);
            can_send_motor_lg(&motorL, &motorR);
        }

        xTaskResumeAll();

        vTaskDelay(10);
    }

}
