#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* 基础配置 */
#define configUSE_PREEMPTION                    1
#define configUSE_MUTEXES                       1
#define configUSE_QUEUE_SETS                    1
#define configCPU_CLOCK_HZ                      ((unsigned long)20000000)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    ( 5 )
#define configMINIMAL_STACK_SIZE                ( 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 20 * 1024 ) ) /* 分配 20K 给 FreeRTOS 堆 */
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* 必须开启这个宏，否则 tasks.c 不会编译 vTaskDelay */
#define INCLUDE_vTaskDelay          1
#define INCLUDE_vTaskPrioritySet    1
#define INCLUDE_vTaskSuspend        1
#define INCLUDE_xTaskDelayUntil     1



/* 开启软件定时器相关配置 */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )


/* --- 补充配置：钩子函数 (Hooks) --- */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            0

/* --- 补充配置：Cortex-M 中断优先级 (关键) --- */
/* LM3S6965 (Cortex-M3) 实现了 3 位中断优先级 */
#define configPRIO_BITS                         3

/* 最低中断优先级 (7) */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      7

/* FreeRTOS 可管理的最高中断优先级 (5)。
 * 优先级高于 5 的中断将不会被 FreeRTOS 屏蔽，但它们也不能调用 FreeRTOS API！*/
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* 转换为 Cortex-M 硬件所需的移位格式 */
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#endif /* FREERTOS_CONFIG_H */