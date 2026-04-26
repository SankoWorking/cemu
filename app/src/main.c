#include "tasks_init.h"
#include "task_logger.h"


int main(void){
    //Init_System();
    Init_Log();
    Start_Log_Task();

    Log_Msg( MOD_SYS, "Hello");
    vTaskStartScheduler();
    return 0;
}