#include "Task_DebugMsg.h"

extern Attitude CloudAttitude;
extern POS_PID_Struct PID_Pitch_P;
extern POS_PID_Struct PID_Yaw_P;
extern DBUS_Type Remoter;
extern DotShootType DotShoot;
extern POS_PID_Struct PID_DotShoot_VO;
extern Ramp_Struct Chasis1_Ramp;
extern EncoderType PitchEncoder;
extern EncoderType YawEncoder;

extern OS_TCB  Task_IMUTCB;
extern OS_TCB  Task_CanTxTCB;
extern OS_TCB  Task_ControlTCB;
extern OS_TCB  Task_MonitorTCB;
extern OS_TCB  Task_DebugMsgTCB;

extern POS_PID_Struct PID_ChasisMotor2;
extern POS_PID_Struct PID_Pitch_P;
extern POS_PID_Struct PID_Pitch_V;
extern POS_PID_Struct PID_Yaw_P;
extern POS_PID_Struct PID_Yaw_V;
extern POS_PID_Struct PID_Servo;
extern float q0;
extern float q1;
extern float q2;
extern float q3;

extern uint16_t rec ;

extern DataTransType VisionPackage[VISION_DATA_LENGTH+2];
extern EncoderType PitchEncoder;
extern EncoderType YawEncoder;

extern int16_t fifo[9];

extern  uint16_t Scan_Cnt;
extern int16_t temp;

void Task_DebugMsg(void *p_arg)
{
	(void )p_arg;
	OS_ERR err;

	for (;;)
	{
		
		OSTimeDly(10,OS_OPT_TIME_DLY,&err);
		
//		if (rec)
//		{
//			a = (YawEncoder.smooth_enc - YawEncoder.bias) * 0.0439453125f;
//			b = (PitchEncoder.smooth_enc - PitchEncoder.bias) * 0.0439453125f;
//			
//			printf("%f %f %f | yaw :%f pitch:%f | %f %f\r\n",VisionPackage[0].Trans,VisionPackage[1].Trans,VisionPackage[2].Trans,
//			a,b, (float)a / VisionPackage[0].Trans,(float)b / VisionPackage[1].Trans);
//			rec = 0;
//		}
		//printf("%f %f\r\n",PID_Yaw_V.Expect,CloudAttitude.ENC_Yaw * 0.0439453125f);
		//printf("%d\r\n",Scan_Cnt);
		//printf("Yaw :%f %f | Pitch :%f %f\r\n",PID_CV_ENEMY_X.Measured,PID_CV_ENEMY_X.SumErr,PID_CV_ENEMY_Y.Measured,PID_CV_ENEMY_Y.SumErr);
		
		
		//printf("%f %f\r\n",VisionPackage[0].Trans,VisionPackage[1].Trans);
		//printf("pitch = %f roll = %f yaw = %f | %d %d %d\r\n",CloudAttitude.pitch,CloudAttitude.roll,CloudAttitude.yaw,fifo[3],fifo[4],fifo[5]);
		printf("yaw = %f pitch:%f roll:%f | Temp :%f\r\n",CloudAttitude.yaw,CloudAttitude.pitch,CloudAttitude.roll,(36.53f + ((double)temp)/340.0f));
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
		SetFricMotor(1);
		//printf("%f %f %f %f\r\n",q0,q1,q2,q3);
		//printf("CPU Used: = %f%% \r\n",(float)OSStatTaskCPUUsage/100.0f);
		//printf("ch0:%d ch1:%d left_switch:%d right_switch:%d\r\n",Remoter.ch0,Remoter.ch1,Remoter.left_switch,Remoter.right_switch);
		//printf(" %d  %ld %f %f\n",DotShoot.Speed,DotShoot.Position,PID_DotShoot_VO.Expect,PID_DotShoot_VO.Measured);
		//DotShootControl();
		//OSTimeDly(100,OS_OPT_TIME_DLY,&err);
//		printf("IMU_Task:%d CanTx_Task:%d Control_Task:%d DebugMsg_Task:%d\r\n",
//						Task_IMUTCB.StkUsed,Task_CanTxTCB.StkUsed,Task_ControlTCB.StkUsed,Task_DebugMsgTCB.StkUsed);
		//printf("Monitor_Task:%d\r\n",Task_MonitorTCB.StkUsed);
	   //Send2Cloud(2000,0);
		//printf("gx = %f gz = %f\r\n",CloudAttitude.gx,CloudAttitude.gz);
    //printf("ENC_Pitch: %f ENC_Yaw: %f\r\n",CloudAttitude.ENC_Pitch* 0.0439453125f,CloudAttitude.ENC_Yaw*0.0439453125f);
		//printf("%d %d\r\n",CloudAttitude.ENC_Yaw,CloudAttitude.ENC_Pitch);
		//PID_Pitch_P.Expect   += (Remoter.ch3 - 1024) * ANGLE_INC_FAC;
		//printf("PitExp:%f  YawExp:%f yawReal: %f\r\n",PID_Pitch_P.Expect,PID_Yaw_P.Expect,CloudAttitude.yaw);
		//printf("%f\r\n",PID_ChasisMotor2.Measured);
		//printf("Yaw_Expect: %f Pitch_Expect: %f\r\n",PID_Yaw_P.Expect,PID_Pitch_P.Expect);
		//printf("Yaw_Expect: %f Real_Yaw: %f\r\n",PID_Yaw_P.Expect,CloudAttitude.yaw);
		//printf("%f %f\r\n",PID_Pitch_V.Expect,PID_Pitch_V.Expect);
		//printf("%f %f\r\n",CloudAttitude.gx,CloudAttitude.ENC_Pitch* 0.0439453125f);
		//printf("%f %d %f\r\n",PID_Yaw_P.Expect,GetRemoterMsg(CH2),CloudAttitude.yaw);
		//printf("ENC_YAW: %d Now_ENC: %d\r\n",CloudAttitude.ENC_Yaw,YawEncoder.now_enc);
		//printf("%f %f\r\n",PID_Yaw_P.Expect,PID_Pitch_P.Expect);
	}
}
