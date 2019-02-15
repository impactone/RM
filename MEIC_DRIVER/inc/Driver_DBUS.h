#ifndef __DRIVER_DBUS_H
#define __DRIVER_DBUS_H


#include "stdint.h"
#include "Config.h"
#include "Driver_PID.h"


typedef struct
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t  left_switch;
	uint8_t  right_switch;
	
	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t l;
		uint8_t r;		
	}mouse;
	
  uint16_t key;
	uint16_t res;	
	
}DBUS_Type;

typedef enum
{
	CH0,
	CH1,
	CH2,
	CH3,
	LEFT_SWITCH,
	RIGHT_SWTICH,
	MOUSE_X,
	MOUSE_Y,
	MOUSE_Z,
	MOUSE_L,
	MOUSE_R,
	KEY,
	RES
}RemoterMsgType;

#define DBUS_LENGTH  18  //遥控器接收长度

extern uint8_t DBUS_BUFF[DBUS_LENGTH];
extern POS_PID_Struct PID_Pitch_P;
extern POS_PID_Struct PID_Yaw_P;

void DBUS_Init(void);
void DBUS_Decode(void);
int16_t GetRemoterMsg(RemoterMsgType code);

#endif
