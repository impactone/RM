#ifndef __CONFIG_H
#define __CONFIG_H

#define CENTRAL_PITCH -654    //注意，过零点和不过零点初始值定义不一样
#define CENTRAL_YAW   920    //注意，过零点和不过零点初始值定义不一样
#define ANGLE_INC_FAC 0.003f  //遥控器角度增量速度

#define MAX_PITCH  15   //最大云台pitch
#define MIN_PITCH  -20  //最小云台pitch
#define MAX_YAW    30   //最大云台yaw
#define MIN_YAW    -30  //最小云台yaw

#define MAX_CHASISI_CURRENT    32767
#define RM_CHASIS_SPEED_FAC    5.0

#define CHASIS_RAMP_TICK_TO_INC 0xF0000   //底盘速度斜坡斜率 0 - 0xF0000000
#define CLOUD_RAMP_TICK_TO_INC  0x500000  //云台启动斜坡斜率 0 - 0xF0000000

#define BULLET_FREQUENCY_V   50  //子弹射频 双环模式下 单位：*10ms/发
#define BULLET_FREQUENCY_VO  10  //子弹射频 单环模式下
#define BULLET_STUCK_TIME    50  //最大卡弹时间 单位：*10ms
#define BULLET_STUCK_ANGLE   60  //在最大卡弹时间内转过的最小角度 单位：编码器线数

#endif

