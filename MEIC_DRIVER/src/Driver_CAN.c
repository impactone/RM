#include "Driver_CAN.h"

EncoderType PitchEncoder;
EncoderType YawEncoder;
extern Attitude CloudAttitude;
extern uint8_t ChasisMotor1FrameRate       ;
extern uint8_t ChasisMotor2FrameRate       ;
extern uint8_t ChasisMotor3FrameRate       ;
extern uint8_t ChasisMotor4FrameRate       ;
extern uint8_t PitchMotorFrameRate         ;
extern uint8_t YawIMUFrameRate             ;
extern uint8_t YawMotorFrameRate           ;
extern uint8_t ChasisControlBoardFrameRate ;
extern POS_PID_Struct PID_ChasisMotor1;
extern POS_PID_Struct PID_ChasisMotor2;
extern POS_PID_Struct PID_ChasisMotor3;
extern POS_PID_Struct PID_ChasisMotor4;

void GMEncoder_Init(void)
{

  PitchEncoder.bias       = CENTRAL_PITCH;
	PitchEncoder.now_enc    = 0;
	PitchEncoder.dif_enc    = 0;
	PitchEncoder.last_enc   = 0;
	PitchEncoder.rount_cnt  = 0;
	PitchEncoder.smooth_enc = 0;

  YawEncoder.bias         = CENTRAL_YAW;
	YawEncoder.now_enc      = 0;
	YawEncoder.dif_enc      = 0;
	YawEncoder.last_enc     = 0;
	YawEncoder.rount_cnt    = 0;
	YawEncoder.smooth_enc   = 0;

}

void ZGYRO_Reset(void)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x404;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x01;
    tx_message.Data[2] = 0x02;
    tx_message.Data[3] = 0x03;
    tx_message.Data[4] = 0x04;
    tx_message.Data[5] = 0x05;
    tx_message.Data[6] = 0x06;
    tx_message.Data[7] = 0x07;
    
    CAN_Transmit(CAN1,&tx_message);	
}

#define h8b(x) (x>>8)
#define l8b(x) ((x)&0xff)
void Send2Cloud(int16_t pitch,int16_t yaw)
{
	  OS_ERR   err;	  
		static CanTxMsg TxMsg;
		TxMsg.StdId = 0x1ff;
		TxMsg.IDE = CAN_Id_Standard;
		TxMsg.RTR = CAN_RTR_Data;
		TxMsg.DLC = 0x08;
		TxMsg.Data[0] = h8b(yaw);
		TxMsg.Data[1] = l8b(yaw);
		TxMsg.Data[2] = h8b(pitch);
		TxMsg.Data[3] = l8b(pitch);
		TxMsg.Data[4] = 0;
		TxMsg.Data[5] = 0;
		TxMsg.Data[6] = 0;
		TxMsg.Data[7] = 0;	
	  OSQPost(&CanMsgQue,&TxMsg,sizeof(TxMsg),OS_OPT_POST_FIFO,&err);
}

void Send2Chasis(int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{
	  OS_ERR   err;
	  static CanTxMsg TxMsg;
		TxMsg.StdId = 0x200;
		TxMsg.IDE = CAN_Id_Standard;
		TxMsg.RTR = CAN_RTR_Data;
		TxMsg.DLC = 0x08;
		TxMsg.Data[0] = h8b(v1);
		TxMsg.Data[1] = l8b(v1);
		TxMsg.Data[2] = h8b(-v2);
		TxMsg.Data[3] = l8b(-v2);
		TxMsg.Data[4] = h8b(v3);
		TxMsg.Data[5] = l8b(v3);
		TxMsg.Data[6] = h8b(-v4);
		TxMsg.Data[7] = l8b(-v4);	
	  OSQPost(&CanMsgQue,&TxMsg,sizeof(TxMsg),OS_OPT_POST_FIFO,&err);	
}

//过零点处理
void PassZeroProcess(EncoderType *encoder,CanRxMsg RxMsg)
{
	encoder->last_enc = encoder->now_enc;
	encoder->now_enc = (int16_t)(RxMsg.Data[0]<<8 | RxMsg.Data[1]);
	encoder->dif_enc = encoder->now_enc - encoder->last_enc;
  
	if (encoder->dif_enc < -7500)
	{
		encoder->rount_cnt++;
	}else if (encoder->dif_enc > 7500)
	{
		encoder->rount_cnt--;
	}
	
	encoder->smooth_enc = encoder->now_enc + encoder->rount_cnt * 8192;
	
}

void CAN1_Process(CanRxMsg RxMsg)
{
	switch (RxMsg.StdId)
	{
		case ZGYRO_ID:
			YawIMUFrameRate++;
			CloudAttitude.yaw = -0.01f*((int32_t)(RxMsg.Data[0]<<24)|(int32_t)(RxMsg.Data[1]<<16) | (int32_t)(RxMsg.Data[2]<<8) | (int32_t)(RxMsg.Data[3])); 
			break;
		default:
			
			break;
	}

}

void CAN2_Process(CanRxMsg RxMsg)
{
	switch (RxMsg.StdId)
	{
		case RM3510_MOTOR1ID:
			ChasisMotor1FrameRate++;
			PID_ChasisMotor1.Measured = -(int16_t)(RxMsg.Data[2]<<8 | RxMsg.Data[3]);
			break;
		case RM3510_MOTOR2ID:
			ChasisMotor2FrameRate++;
			PID_ChasisMotor2.Measured = (int16_t)(RxMsg.Data[2]<<8 | RxMsg.Data[3]);
			break;
		case RM3510_MOTOR3ID:
			ChasisMotor3FrameRate++;
		  PID_ChasisMotor3.Measured = -(int16_t)(RxMsg.Data[2]<<8 | RxMsg.Data[3]);
			break;
		case RM3510_MOTOR4ID:
			ChasisMotor4FrameRate++;
		  PID_ChasisMotor4.Measured = (int16_t)(RxMsg.Data[2]<<8 | RxMsg.Data[3]);
			break;
		case RM6623_PITCHID:
			PitchMotorFrameRate++;
			PassZeroProcess(&PitchEncoder,RxMsg);
			CloudAttitude.ENC_Pitch = PitchEncoder.smooth_enc - PitchEncoder.bias;
			//CloudAttitude.ENC_Pitch = (int16_t)(RxMsg.Data[0]<<8 | RxMsg.Data[1]) - PitchEncoder.bias;
			break;
		case RM6623_YAWID:
			YawMotorFrameRate++;
			PassZeroProcess(&YawEncoder,RxMsg);
			CloudAttitude.ENC_Yaw   = YawEncoder.smooth_enc - YawEncoder.bias;
			
			//CloudAttitude.ENC_Yaw   = (int16_t)(RxMsg.Data[0]<<8 | RxMsg.Data[1]) - YawEncoder.bias;;
			break;
		default:
			
			break;
	}

}
