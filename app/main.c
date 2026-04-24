#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"

// 简单的任务函数
void vTaskTest(void *pvParameters) {
    for (;;) {
        uart_puts("Task is running!\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    uart_puts("Starting FreeRTOS...\n");

    xTaskCreate(vTaskTest, "TestTask", 128, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1);
}