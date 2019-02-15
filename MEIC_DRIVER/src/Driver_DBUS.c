#include "Driver_DBUS.h"

uint8_t DBUS_BUFF[DBUS_LENGTH];
DBUS_Type Remoter;

void DBUS_Init(void)
{
	Remoter.ch0 = 1024;
	Remoter.ch1 = 1024;
	Remoter.ch2 = 1024;
	Remoter.ch3 = 1024;
	Remoter.left_switch = 0;
	Remoter.right_switch = 0;
	
	Remoter.mouse.x = 0;
	Remoter.mouse.y = 0;
	Remoter.mouse.z = 0;
	Remoter.mouse.l = 0;
	Remoter.mouse.r = 0;
	
	Remoter.key = 0;
	Remoter.res = 0;
}

int16_t GetRemoterMsg(RemoterMsgType code)
{
	int16_t tmp = 0;
	switch (code)
	{
		case CH0:
			tmp = Remoter.ch0;
			break;
		case CH1:
			tmp = Remoter.ch1;
			break;
		case CH2:
			tmp = Remoter.ch2;
			break;
		case CH3:
			tmp = Remoter.ch3;
			break;
		case LEFT_SWITCH:
			tmp = Remoter.left_switch;
			break;
		case RIGHT_SWTICH:
			tmp = Remoter.right_switch;
			break;
		case MOUSE_X:
			tmp = Remoter.mouse.x;
			break;
		case MOUSE_Y:
			tmp = Remoter.mouse.y;
			break;
		case MOUSE_Z:
			tmp = Remoter.mouse.z;
			break;
		case MOUSE_L:
			tmp = Remoter.mouse.l;
			break;
		case MOUSE_R:
		  tmp = Remoter.mouse.r;
		  break;
		case KEY:
			tmp = Remoter.key;
			break;
		case RES:
			tmp = Remoter.res;
			break;
		default:
			break;
	}
	return tmp;
}

/*Ò£¿ØÆ÷½âÂë*/
void DBUS_Decode(void)
{
	Remoter.ch0 = (DBUS_BUFF[0] | (DBUS_BUFF[1] << 8)) & 0x07ff;
	Remoter.ch1 = ((DBUS_BUFF[1] >> 3) | (DBUS_BUFF[2] << 5)) & 0x07ff;
	Remoter.ch2 = ((DBUS_BUFF[2] >> 6) | (DBUS_BUFF[3] << 2) | (DBUS_BUFF[4] << 10)) & 0x07ff;
	Remoter.ch3 = ((DBUS_BUFF[4] >> 1) | (DBUS_BUFF[5] << 7)) & 0x07ff;
	Remoter.left_switch  = ((DBUS_BUFF[5] >> 4) & 0x000C) >> 2;  
	Remoter.right_switch = ((DBUS_BUFF[5] >> 4) & 0x0003); 
	
	Remoter.mouse.x = DBUS_BUFF[6]  | (DBUS_BUFF[7] << 8); 
	Remoter.mouse.y = DBUS_BUFF[8]  | (DBUS_BUFF[9] << 8);
	Remoter.mouse.z = DBUS_BUFF[10] | (DBUS_BUFF[11] << 8);
	Remoter.mouse.l = DBUS_BUFF[12];
	Remoter.mouse.r = DBUS_BUFF[13];
	
	Remoter.key = DBUS_BUFF[14] | (DBUS_BUFF[15] << 8);
	Remoter.res = DBUS_BUFF[16] | (DBUS_BUFF[17] << 8);	

}
	

