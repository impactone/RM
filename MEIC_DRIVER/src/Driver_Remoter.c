#include "Driver_Remoter.h"
#include "Driver_PID.h"

ControlModeType ControlMode;

void Mode_Init(void)
{
	ControlMode = STMode;
}

void ControlModeSwitch(void)
{
	switch (GetRemoterMsg(RIGHT_SWTICH))
	{
		case 1:
			ControlMode = RMMode;
			break;
		case 2:
			ControlMode = STMode;
			break;
		case 3:
			ControlMode = KMMode;
			break;
		default:
			break;
	}
}

ControlModeType GetControlMode(void)
{
	return ControlMode;
}


