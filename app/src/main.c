#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "uart.h" // 假设你已有的串口驱动

// 1. 定义队列句柄
QueueHandle_t xTestQueue;

// 任务 A：生产者 (发送数据)
void vSenderTask(void *pvParameters) {
    int32_t lValueToSend = 0;
    BaseType_t xStatus;

    for(;;) {
        lValueToSend++; // 数据自增
        
        // 2. 发送数据到队列
        // 参数：队列句柄, 数据地址, 等待时间(0代表不等待)
        xStatus = xQueueSend(xTestQueue, &lValueToSend, 0);

        if(xStatus == pdPASS) {
            uart_puts("Sender: Sent value to queue\n");
        } else {
            uart_puts("Sender: Could not send to queue (Full!)\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒发送一次
    }
}

// 任务 B：消费者 (接收数据)
void vReceiverTask(void *pvParameters) {
    int32_t lReceivedValue;
    BaseType_t xStatus;

    for(;;) {
        // 3. 从队列接收数据
        // 参数：队列句柄, 存放数据的缓冲区, 等待时间(portMAX_DELAY代表死等)
        xStatus = xQueueReceive(xTestQueue, &lReceivedValue, portMAX_DELAY);

        if(xStatus == pdPASS) {
            uart_puts("Receiver: Got data from queue. Value = ");
            //uart_puts(lReceivedValue);
            // 这里可以添加格式化输出 Value 的代码
            uart_puts("Success\n");
        }
    }
}

int main(void) {
    // 4. 创建队列：能够存储 5 个 int32_t 类型的数据
    xTestQueue = xQueueCreate(5, sizeof(int32_t));

    if(xTestQueue != NULL) {
        // 5. 创建任务
        // 接收者优先级设高一点，确保数据一到就能立刻处理
        xTaskCreate(vReceiverTask, "Receiver", 1000, NULL, 2, NULL);
        xTaskCreate(vSenderTask, "Sender", 1000, NULL, 1, NULL);

        // 6. 启动调度器
        vTaskStartScheduler();
    } else {
        // 队列创建失败（通常是堆空间不足）
        for(;;);
    }

    return 0;
}