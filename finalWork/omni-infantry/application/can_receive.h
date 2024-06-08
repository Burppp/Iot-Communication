//
// Created by xhuanc on 2021/9/27.
//
/*轮子电机id：                                 解算坐标：      x(前)
            ****      前       ****                                 |
           * 2 *             * 1 *                                 |
            ****              ****                                 |
                                                                   |
           左                   右                    --------------z-----------y(右)
                                                                   |
            ****              ****                                 |
           * 3 *            * 4  *                                 |
            ****      后      ****                                 |

*/

#ifndef _CAN_RECEIVE_H_
#define _CAN_RECEIVE_H_
#include "struct_typedef.h"
#include "PID.h"
#include "Cap.h"
/******************** define *******************/

#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8192
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define MOTOR_ECD_TO_ANGLE 0.0439453125f     //        360  /8192
#define MOTOR_RAD_TO_ANGLE 57.29577951308238f // 360*2PI
#define MOTOR_ANGLE_TO_RAD  0.0174532925199433f
/******************** struct *******************/

//CAN_ID 该枚举不区分CAN_1还是CAN_2
typedef enum
{

    //电机控制 发送ID
    CAN_MOTOR_0x200_ID = 0x200,
    CAN_MOTOR_0x1FF_ID = 0x1FF,
    CAN_MOTOR_0x2FF_ID = 0x2FF,

    //0X200对应的电机ID(CAN1)
    CAN_CHASSIS_3508_MOTOR_RF=0x202,
    CAN_CHASSIS_3508_MOTOR_LF=0x201,
    CAN_CHASSIS_3508_MOTOR_LB=0x204,
    CAN_CHASSIS_3508_MOTOR_RB=0x203,

    //0X1FF对应的电机ID(CAN1)
    CAN_GIMBAL_6020_YAW=0x205,
//    CAN_LAUNCHER_2006_BARREL=0x206,//test

    //0X200对应的电机ID(CAN2)
    //0X1FF对应的电机ID(CAN2)
    CAN_LAUNCHER_3508_FIRE_R=0X205,//这是摩擦轮
    CAN_GIMBAL_6020_PITCH=0x206,
    CAN_LAUNCHER_3508_FIRE_L=0X207,//这是摩擦轮
    CAN_LAUNCHER_2006_TRIGGER=0x208,


} can_msg_id_e;

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

//电机的数据
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

    int32_t round_cnt;   //电机旋转的总圈数
    int32_t total_ecd;   //电机旋转的总编码器数值

    uint16_t offset_ecd;//电机的校准编码值

    int32_t total_dis;   //电机总共走的距离

    float torque_round_cnt;//转子转过的角度（-360-360）
    float real_round_cnt;//轮子转过的角度
    float real_angle_deg;
} motor_measure_t;

typedef struct
{
    const motor_measure_t *motor_measure;

    fp32 speed;

    fp32 rpm_set;

    pid_t speed_p;

    int16_t give_current;


}motor_3508_t;

typedef struct
{

    motor_measure_t *motor_measure;
    //自然状态下pid

    pid_t angle_p;

    pid_t speed_p;
    //自瞄pid

    pid_t angle_p_auto;

    pid_t speed_p_auto;


    fp32 max_relative_angle; //°

    fp32 min_relative_angle; //°

    fp32 relative_angle_get;
    fp32 relative_angle_set; //°

    fp32 absolute_angle_get;
    fp32 absolute_angle_set;//rad

    fp32 gyro_set;  //转速设置
    int16_t give_current; //最终电流值

}motor_6020_t;


typedef struct
{
    motor_measure_t *motor_measure;

    pid_t angle_p;//角度环pid

    pid_t speed_p;//速度环pid

    fp32 speed;//转速期望值

    int16_t give_current;

}motor_2006_t;

//typedef struct {
//    union {
//        float value;
//        uint8_t data[4];
//    }vx;
//
//    union {
//        float value;
//        uint8_t data[4];
//    }vy;
//}Vector_get_from_top;

typedef union
{
    uint8_t original_data[3];
    uint32_t distance_data;
}tof_t;

/******************** extern *******************/



extern motor_measure_t motor_3508_measure[6];
extern motor_measure_t motor_yaw_measure;
extern motor_measure_t motor_pitch_measure;
extern motor_measure_t motor_shoot_measure[4];

extern void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void CAN_cmd_chassis_rudder(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd);

extern fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd);

extern void CAN_cmd_communication(CAN_TYPE can_type,can_msg_id_e CMD_ID,fp32 vx,fp32 vy);

extern void CAN_cmd_cap2(cap2_info_t*cap);

#endif
