#ifndef __TASKS_INIT_H__
#define __TASKS_INIT_H__

#include "tasks_interfaces.h"
#include "FreeRTOS.h"
#include "attitude.h"
#include "sensors.h"
#include "commands.h"
#include "uart.h"

void Init_System(void);

#endif /* #ifndef __TASKS_INIT_H__ */