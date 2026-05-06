#ifndef MAHONYAHRS_H
#define MAHONYAHRS_H

#include <math.h>

#define sampleFreq	100.0f		

// 比例增益 (Kp) 和积分增益 (Ki)
// Kp 越大，姿态越信任加速度计/磁力计的测量值 (收敛快，但容易受高频震动干扰)
// Ki 用于消除陀螺仪的零点漂移
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

// 全局变量声明：四元数 (供外部的 Quaternion_To_Euler 函数读取)
extern volatile float q0, q1, q2, q3;

// 算法初始化/复位
void MahonyAHRSinit(void);

// 9轴姿态解算 (融合 陀螺仪 + 加速度计 + 磁力计)
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

// 6轴姿态解算 (仅融合 陀螺仪 + 加速度计，无磁力计环境退化使用)
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif