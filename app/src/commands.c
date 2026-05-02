#include "commands.h"

/*
 *  心跳包发送任务，每1ms向14580端口发送心跳包。
 */
static void Task_Heartbeat(void *pvParameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    const TickType_t Frequency = pdMS_TO_TICKS(1000);
    uint8_t Buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t Msg;
    for(;;) {
        vTaskDelayUntil(&LastWakeTime, Frequency);
        Puts_UART0("Heartbeat Woke Up!\r\n");
        mavlink_msg_heartbeat_pack(
            1,
            MAV_COMP_ID_ONBOARD_COMPUTER,
            &Msg, 
            MAV_TYPE_ONBOARD_CONTROLLER,
            MAV_AUTOPILOT_INVALID,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
            0,
            MAV_STATE_ACTIVE
        );
        uint16_t Len = mavlink_msg_to_send_buffer(Buffer, &Msg);
        Send_UART1(Buffer, Len);
    }
}

void Init_Heartbeat_Tasks(void) {
    xTaskCreate(
        Task_Heartbeat,
        "Heartbeat",
        STACK_COMMAND_PARSE,
        NULL,
        PRIO_COMMAND_PARSE_TASK,
        NULL
    );
}