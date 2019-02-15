#ifndef __DRIVER_REMOTER_H
#define __DRIVER_REMOTER_H

#include "Driver_DBUS.h"

typedef enum
{
	RMMode,
	KMMode,
	STMode
}ControlModeType;

void Mode_Init(void);
void ControlModeSwitch(void);
ControlModeType GetControlMode(void);

#endif

