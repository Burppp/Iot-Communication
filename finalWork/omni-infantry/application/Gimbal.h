//
// Created by xhuanc on 2021/10/13.
//

#ifndef _GIMBAL_H_
#define _GIMBAL_H_
/*      Include     */

#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"
#include "cmsis_os.h"
#include "launcher.h"
#include "can_receive.h"
#include "user_lib.h"
#include "protocol_shaob.h"
#include "filter.h"
#include "Detection.h"
#include "packet.h"
#include "Chassis.h"
/*      define     */

#define GIMBAL_TASK_INIT_TIME 3000
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

//pitch最大最小绝对角
#define GIMBAL_PITCH_MAX_ABSOLUTE_ANGLE (30)
#define GIMBAL_PITCH_MIN_ABSOLUTE_ANGLE (-20)

//pitch yaw 初始ecd值
#define GIMBAL_PITCH_OFFSET_ECD 7516
#define GIMBAL_YAW_OFFSET_ECD 987
//#define GIMBAL_YAW_OFFSET_ECD 7548

#define RC_TO_YAW 0.001f
#define RC_TO_PITCH 0.0008f
//最大最小绝对相对角度
#define MAX_ABS_ANGLE (30)
#define MIN_ABS_ANGLE (-30)
#define MAX_RELA_ANGLE (20)//35
#define MIN_RELA_ANGLE (-31)//-20
//云台转动速度系数
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f//0.5f
#define GIMBAL_RC_MOVE_RATIO_YAW 0.8f//0.8f
#define GIMBAL_YAW_PATROL_SPEED 0.08f



#define GIMBAL_PITCH_PATROL_SPEED 0.06f
//普通云台pid常量
#define GIMBAL_YAW_ANGLE_PID_KP     100.f//50f
#define GIMBAL_YAW_ANGLE_PID_KI     0.1f//0.05f
#define GIMBAL_YAW_ANGLE_PID_KD     1500.0f//800.f
#define GIMBAL_YAW_ANGLE_MAX_OUT    10000.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   100.f//

#define GIMBAL_YAW_SPEED_PID_KP     200.f//
#define GIMBAL_YAW_SPEED_PID_KI     3.f//20.5f
#define GIMBAL_YAW_SPEED_PID_KD     0.0f//80
#define GIMBAL_YAW_SPEED_MAX_OUT    30000.f
#define GIMBAL_YAW_SPEED_MAX_IOUT   3000.f


#define GIMBAL_PITCH_ANGLE_PID_KP   25.f
#define GIMBAL_PITCH_ANGLE_PID_KI   0.005f//1
#define GIMBAL_PITCH_ANGLE_PID_KD   600.f//280
#define GIMBAL_PITCH_ANGLE_MAX_OUT  400.f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT 40.f

#define GIMBAL_PITCH_SPEED_PID_KP   140.f//300
#define GIMBAL_PITCH_SPEED_PID_KI   1.5f//5
#define GIMBAL_PITCH_SPEED_PID_KD   100.f//40
#define GIMBAL_PITCH_SPEED_MAX_OUT  30000.f//30000
#define GIMBAL_PITCH_SPEED_MAX_IOUT 4000.f//13000



//自瞄pid常量
//yaw

#define GIMBAL_YAW_ANGLE_PID_KP_AUTO     100.f//15f
#define GIMBAL_YAW_ANGLE_PID_KI_AUTO     0.1f//0.005f
#define GIMBAL_YAW_ANGLE_PID_KD_AUTO     2000.0f//300.f
#define GIMBAL_YAW_ANGLE_MAX_OUT_AUTO    10000.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT_AUTO   100.f//60

#define GIMBAL_YAW_SPEED_PID_KP_AUTO     200.f//180
#define GIMBAL_YAW_SPEED_PID_KI_AUTO     3.f//50.0f
#define GIMBAL_YAW_SPEED_PID_KD_AUTO     0.0f//
#define GIMBAL_YAW_SPEED_MAX_OUT_AUTO    30000.f
#define GIMBAL_YAW_SPEED_MAX_IOUT_AUTO   3000.f

//pitch
#define GIMBAL_PITCH_ANGLE_PID_KP_AUTO   25.f
#define GIMBAL_PITCH_ANGLE_PID_KI_AUTO   0.05f
#define GIMBAL_PITCH_ANGLE_PID_KD_AUTO   600.f
#define GIMBAL_PITCH_ANGLE_MAX_OUT_AUTO  500.f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT_AUTO 60.f

#define GIMBAL_PITCH_SPEED_PID_KP_AUTO   100.f
#define GIMBAL_PITCH_SPEED_PID_KI_AUTO   3.5f
#define GIMBAL_PITCH_SPEED_PID_KD_AUTO   100.f
#define GIMBAL_PITCH_SPEED_MAX_OUT_AUTO  30000.f
#define GIMBAL_PITCH_SPEED_MAX_IOUT_AUTO 9000.f


/*      结构体和枚举     */

typedef enum {
    GIMBAL_RELAX=0,//云台失能
    GIMBAL_BACK,//云台回中
    GIMBAL_ACTIVE,
    GIMBAL_AUTO,
    GIMBAL_BUFF, //大符
    GIMBAL_SBUFF, //小符
    GIMBAL_FORECAST,//预测
}gimbal_mode_e;

typedef struct {
    motor_6020_t yaw;
    motor_6020_t pitch;
    motor_2006_t trigger;
    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

//    AHRS_Eulr_t*Eulr;   //姿态角

    fp32 relative_gyro_yaw;
    fp32 absolute_gyro_yaw;
    bool_t yaw_is_back;
    fp32 relative_gyro_pitch;
    fp32 absolute_gyro_pitch;
    bool_t pitch_is_back;
    int32_t yaw_imu_offset_angle;
    float horizon_angle;

}gimbal_t;

_Noreturn extern void gimbal_task(void const*pvParameters);


#endif
