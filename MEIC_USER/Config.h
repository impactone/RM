#ifndef __CONFIG_H
#define __CONFIG_H

#define CENTRAL_PITCH -654    //ע�⣬�����Ͳ�������ʼֵ���岻һ��
#define CENTRAL_YAW   920    //ע�⣬�����Ͳ�������ʼֵ���岻һ��
#define ANGLE_INC_FAC 0.003f  //ң�����Ƕ������ٶ�

#define MAX_PITCH  15   //�����̨pitch
#define MIN_PITCH  -20  //��С��̨pitch
#define MAX_YAW    30   //�����̨yaw
#define MIN_YAW    -30  //��С��̨yaw

#define MAX_CHASISI_CURRENT    32767
#define RM_CHASIS_SPEED_FAC    5.0

#define CHASIS_RAMP_TICK_TO_INC 0xF0000   //�����ٶ�б��б�� 0 - 0xF0000000
#define CLOUD_RAMP_TICK_TO_INC  0x500000  //��̨����б��б�� 0 - 0xF0000000

#define BULLET_FREQUENCY_V   50  //�ӵ���Ƶ ˫��ģʽ�� ��λ��*10ms/��
#define BULLET_FREQUENCY_VO  10  //�ӵ���Ƶ ����ģʽ��
#define BULLET_STUCK_TIME    50  //��󿨵�ʱ�� ��λ��*10ms
#define BULLET_STUCK_ANGLE   60  //����󿨵�ʱ����ת������С�Ƕ� ��λ������������

#endif

