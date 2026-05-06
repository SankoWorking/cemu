#include "pid.h"


float Pid_Compute(PID_Controller_t *Pid, float Target, float Measure, float Dt) {
    float Error = Target - Measure;
    float POut = Pid->Kp * Error;

    Pid->Integral += Error * Dt;
    if (Pid->Integral > Pid->IntegralLimit) Pid->Integral = Pid->IntegralLimit;
    else if (Pid->Integral < -Pid->IntegralLimit) Pid->Integral = -Pid->IntegralLimit;
    float IOut = Pid->Ki * Pid->Integral;

    float DOut = Pid->Kd * (Error - Pid->LastError) / Dt;
    Pid->LastError = Error;

    float Output = POut + IOut + DOut;
    if (Output > Pid->OutputLimit) Output = Pid->OutputLimit;
    else if (Output < -Pid->OutputLimit) Output = -Pid->OutputLimit;

    return Output;
}