#ifndef __TASK_COMMANDS_H__
#define __TASK_COMMANDS_H__

#include "tasks_config.h"
#include "uart.h"
#include "queue.h"
#include <common/mavlink.h>
#include "tasks_interfaces.h"
#include "task.h"

/*
 *  初始化心跳包发送任务
 */
void Init_Command_Tasks(void);
#endif