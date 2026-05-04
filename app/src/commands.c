#include "commands.h"
#include "tasks_interfaces.h" 

/**
 * @brief 心跳包发送任务 (保持发送，以维持链路连接)
 */
static void Task_Heartbeat(void *PvParameters) {
    uint8_t Buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t Msg;
    uint64_t LastTime = 0;

    for(;;) {
        // 降低检查频率，每 100ms 唤醒一次
        vTaskDelay(pdMS_TO_TICKS(100));

        if (CurrentImuData.Timestamp != 0 && (CurrentImuData.Timestamp - LastTime >= 500000)) {
            LastTime = CurrentImuData.Timestamp;
            
            // 注意：我们作为主飞控，SystemID=1, CompID=1
            mavlink_msg_heartbeat_pack(
                1, 1, &Msg, 
                MAV_TYPE_QUADROTOR,    // 无人机类型
                MAV_AUTOPILOT_GENERIC, // 通用飞控
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED, // 设为解锁状态
                0, 
                MAV_STATE_ACTIVE
            );
            
            uint16_t Len = mavlink_msg_to_send_buffer(Buffer, &Msg);
            Send_UART1(Buffer, Len);
        }
    }
}

/**
 * @brief 电机指令发送任务 (直接 HIL 模式)
 * @note  直接向 Gazebo 物理引擎发送 HIL_ACTUATOR_CONTROLS
 */
static void Task_Motor_Sender(void *PvParameters) {
    MotorCommandMsg_t MotorMsg;
    mavlink_message_t MavMsg;
    uint8_t Buffer[MAVLINK_MAX_PACKET_LEN];
    
    // HIL_ACTUATOR_CONTROLS 需要 16 个通道的数组
    float Controls[16]; 
    
    static uint8_t BootStep = 0;

    for (;;) {
        // 等待 Attitude 任务算完 PID 后产生的通知
        if (xQueueReceive(MotorControlQueue, &MotorMsg, portMAX_DELAY) == pdPASS) {
            
            // 初始化所有通道为 -1.0 (底层控制器的停转/最小输出值)
            for (int Index = 0; Index < 16; Index++) { 
                Controls[Index] = -1.0f; 
            }

            // 填充前四个电机的动力 (PID 算出的 0.0 ~ 1.0)
            Controls[0] = MotorMsg.MotorOutputs[0]; // Roll
            Controls[1] = MotorMsg.MotorOutputs[1]; // Pitch
            Controls[2] = MotorMsg.MotorOutputs[2]; // Yaw
            Controls[3] = MotorMsg.MotorOutputs[3]; // Thrust (油门)

            // 💡 直接 HIL 控制包：直接发送给 Gazebo 物理引擎
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