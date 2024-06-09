//
// Created by xhuanc on 2021/10/10.
//

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

/*include*/
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "can_receive.h"
#include "PID.h"
#include "user_lib.h"
#include "queue.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "ramp.h"
#include "bsp_buzzer.h"
#include "math.h"

/*define*/
//底盘在motor_3508_measure中的标号
typedef enum {
    RF=0,
    LF,
    LB,
    RB
}chassis_motor_index_e;

//任务开始空闲一段时间

#define CHASSIS_TASK_INIT_TIME 157

#define CHASSIS_3508_PID_KP     7.0f
#define CHASSIS_3508_PID_KI     0.0f//1.0f
#define CHASSIS_3508_PID_KD     0.0f
#define CHASSIS_3508_PID_MAX_OUT 8000.0f
#define CHASSIS_3508_PID_MAX_IOUT 1000.0f

#define chassis_start_buzzer buzzer_on  (31, 19999)
#define chassis_buzzer_off   buzzer_off()            //buzzer off，关闭蜂鸣器

//底盘机械信息 /m
#define Wheel_axlespacing 0.448f //H
#define Wheel_spacing 0.391f //W
#define GIMBAL_OFFSET 0
#define PERIMETER 0.47414f //轮子周长 /m
#define M3508_DECELE_RATIO (1.0f/14.0f)//1：14 3508减速比

typedef struct
{
    motor_3508_t motor_chassis[4];
    QueueHandle_t motor_data_queue;

    pid_t chassis_vw_pid;
    fp32 vx;
    fp32 vy;
    fp32 vw;

    fp32 vx_pc;
    fp32 vy_pc;
    fp32 vw_pc;

} chassis_t;

//函数声明
_Noreturn extern void chassis_task(void const *pvParameters);
void chassis_speed_update();
void chassis_wheel_cal();
//
#endif

