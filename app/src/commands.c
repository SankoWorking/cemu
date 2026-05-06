#include "commands.h"
#include "tasks_interfaces.h" 

/**
 * @brief 心跳包发送任务,每隔100ms被唤醒，检查仿真世界的时间是否到达了预定的发送时间。
 */
static void Task_Heartbeat(void *PvParameters) {
    uint8_t Buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t Msg;
    uint64_t LastTime = 0;

    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (CurrentImuData.Timestamp != 0 && (CurrentImuData.Timestamp - LastTime >= 500000)) {
            LastTime = CurrentImuData.Timestamp;
            
            mavlink_msg_heartbeat_pack(
                1, 1, &Msg, 
                MAV_TYPE_QUADROTOR,
                MAV_AUTOPILOT_GENERIC,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
                0, 
                MAV_STATE_ACTIVE
            );
            
            uint16_t Len = mavlink_msg_to_send_buffer(Buffer, &Msg);
            Send_UART1(Buffer, Len);
        }
    }
}

/**
 * @brief 电机指令发送任务
 */
static void Task_Motor_Sender(void *PvParameters) {
    MotorCommandMsg_t MotorMsg;
    mavlink_message_t MavMsg;
    uint8_t Buffer[MAVLINK_MAX_PACKET_LEN];

    float Controls[16]; 
    
    static uint8_t BootStep = 0;

    for (;;) {
        // 等待 Attitude 任务算完 PID 后产生的通知
        if (xQueueReceive(MotorControlQueue, &MotorMsg, portMAX_DELAY) == pdPASS) {
            float M1 = MotorMsg.MotorOutputs[0];
            float M2 = MotorMsg.MotorOutputs[1];
            float M3 = MotorMsg.MotorOutputs[2];
            float M4 = MotorMsg.MotorOutputs[3];

            

            for (int Index = 0; Index < 16; Index++) { 
                Controls[Index] = 0.0f; 
            }

            Controls[0] = M1; 
            Controls[1] = M2; 
            Controls[2] = M3; 
            Controls[3] = M4; 

            // 6. 打包并通过网桥发送给仿真世界
            mavlink_msg_hil_actuator_controls_pack(
                1,                          // System ID
                1,                          // Component ID
                &MavMsg,
                MotorMsg.TimestampUs,       // 使用从网桥传来的 Gazebo 同步时间戳
                Controls,
                MAV_MODE_FLAG_SAFETY_ARMED, // 直接声明处于解锁状态
                0                           // Flags
            );

            uint16_t Len = mavlink_msg_to_send_buffer(Buffer, &MavMsg);
            Send_UART1(Buffer, Len);

            // 仅用于调试打印一次起飞信号
            if (BootStep == 0 && MotorMsg.TimestampUs > 2000000) {
                Puts_UART0("HIL Mode: Motor Commands Outputting...\r\n");
                BootStep = 1;
            }
        }
    }
}

void Init_Command_Tasks(void) {
    xTaskCreate(Task_Heartbeat, "Heartbeat", STACK_COMMAND_PARSE, NULL, PRIO_COMMAND_PARSE_TASK, NULL);
    xTaskCreate(Task_Motor_Sender, "MotorSender", STACK_COMMAND_PARSE, NULL, PRIO_COMMAND_PARSE_TASK, NULL);
}