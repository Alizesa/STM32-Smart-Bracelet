/**
  * 抬腕检测 + 跌倒检测
  *
  * 基于 MPU6050 加速度计数据 (满量程 +/-16g, 1g = 2048 LSB):
  *   - 抬腕: 腕部姿态角由"大角度下垂"快速变为"面向使用者"并保持
  *           一段时间 -> 判定为一次抬腕.
  *   - 跌倒: 先出现自由落体(合成加速度接近0), 紧接着出现落地冲击
  *           (>3g), 随后约1.5s内保持静止(加速度稳定在1g附近)
  *           -> 判定为跌倒.
  *
  * 硬件中断(需把MPU6050 INT脚飞线到 PinMap.h 的 MPU6050_INT_*):
  *   MPU6050 在检测到运动/自由落体时会在 INT 脚输出脉冲, 触发 EXTI。
  *   中断里只置标志位(不做I2C), 由 Sensor_Task 周期读到后读取
  *   INT_STATUS 确认来源。保留连续采样作软件确认层(计步/可靠性)。
  */
#include "stm32f10x.h"
#include <math.h>
#include "Motion.h"
#include "MPU6050.h"
#include "PinMap.h"
#include "FreeRTOS.h"
#include "task.h"

#define PI_F                3.14159265f
#define GRAVITY_LSB         2048.0f     /* +/-16g 量程下 1g = 2048 LSB */

/* ---- 抬腕阈值 (绝对俯仰角, 单位度) ---- */
#define RAISE_DOWN_DEG      50.0f       /* 大于此角度视为手臂下垂 */
#define RAISE_UP_DEG        25.0f       /* 小于此角度视为面向使用者 */
#define RAISE_DWELL_MS      200         /* 保持面向使用者 200ms 才确认 */

/* ---- 跌倒阈值 (LSB) ---- */
#define FALL_FREE_LSB       900         /* < ~0.44g 视为失重 */
#define FALL_IMPACT_LSB     4200        /* > ~2.0g 视为撞击 */
#define FALL_FREE_MS        40          /* 失重需持续 40ms */
#define FALL_STILL_MS       800         /* 冲击后静止观察窗口 */
#define FALL_STILL_LSB      600         /* 窗口内偏离1g的容差 */
#define FALL_IMPACT_WINDOW_MS 700       /* 剧烈运动到撞击的最大间隔 */
#define FALL_IMPACT_DELTA_LSB 3000      /* 20ms内约1.5g的三轴突变视为冲击 */
#define FALL_CLEAR_MS       8000        /* 跌倒告警自动复位时间 */

/* ---- 运动检测 (用于运动亮屏/静止自动息屏) ---- */
#define MOTION_DELTA_LSB    150         /* 相邻两帧三轴加速度变化量之和超过此值视为"在动" */

static int16_t PitchDeg10;
static int16_t RollDeg10;
static uint8_t  Raised;
static uint8_t  FallAlarm;
static uint32_t FallAlarmTime;

/* MPU6050硬件中断(INT脚)标志与自由落体事件时间戳 */
static volatile uint8_t MotionIntFlag;
static uint32_t HwFreeFallMs;

/* 运动检测状态(相邻采样加速度变化) */
static uint32_t LastMotionMs;
static uint32_t LastStrongImpactMs;
static int16_t PrevAx, PrevAy, PrevAz;
static uint8_t HavePrevSample;

void Motion_Init(void)
{
	PitchDeg10 = 0;
	RollDeg10 = 0;
	Raised = 0;
	FallAlarm = 0;
	FallAlarmTime = 0;
	MotionIntFlag = 0;
	HwFreeFallMs = 0;
	LastMotionMs = 0;
	LastStrongImpactMs = 0;
	HavePrevSample = 0;
	PrevAx = PrevAy = PrevAz = 0;
}

/**
  * @brief 初始化MPU6050 INT引脚对应的EXTI(抬腕/跌倒硬件中断).
  * @note  需先把MPU6050的INT脚飞线到 PinMap.h 中 MPU6050_INT_* 指定的引脚(默认PB12)。
  *        使用上升+下降双边沿, 兼容高/低有效两种INT输出。
  */
void Motion_IntPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(MPU6050_INT_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);

	/* 上拉输入: 推挽/开漏两种INT输出都能正确读到电平 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = MPU6050_INT_PIN;
	GPIO_Init(MPU6050_INT_PORT, &GPIO_InitStructure);

	GPIO_EXTILineConfig(MPU6050_INT_PORT_SOURCE, MPU6050_INT_PIN_SOURCE);

	EXTI_InitStructure.EXTI_Line = MPU6050_INT_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = MPU6050_INT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;   /* >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5), 否则FreeRTOS断言失败 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief EXTI15_10中断服务函数(由 stm32f10x_it.c 调用)
  * @note  中断内只清EXTI标志并置位, 不做I2C/RTOS调用, 读取INT_STATUS交给任务
  */
void Motion_EXTI_IRQHandler(void)
{
	if (EXTI_GetITStatus(MPU6050_INT_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(MPU6050_INT_EXTI_LINE);
		MotionIntFlag = 1;
	}
}

/**
  * @brief 抬腕状态机
  * @param  pitch   当前俯仰角(度)
  * @param  now_ms  当前时间 ms
  */
static void Wrist_Update(float pitch, uint32_t now_ms)
{
	static uint8_t state = 0;       /* 0=下垂 1=过渡 2=面向使用者 */
	static uint32_t upSince = 0;
	float ap = (float)fabsf(pitch);

	if (ap > RAISE_DOWN_DEG)
	{
		state = 0;
		Raised = 0;
	}
	else if (ap < RAISE_UP_DEG)
	{
		if (state != 2)
		{
			state = 2;
			upSince = now_ms;
		}
		if ((uint32_t)(now_ms - upSince) >= RAISE_DWELL_MS)
		{
			Raised = 1;
		}
	}
	else
	{
		state = 1;
	}
}

static void Fall_Update(float mag, uint32_t now_ms)
{
	static uint8_t armed = 0;
	static uint32_t armTime = 0;
	static uint8_t postImpact = 0;
	static uint32_t postImpactTime = 0;
	static int32_t maxDev = 0;

	if (FallAlarm && (uint32_t)(now_ms - FallAlarmTime) >= FALL_CLEAR_MS)
	{
		FallAlarm = 0;
	}

	if (mag < FALL_FREE_LSB)
	{
		if (!armed)
		{
			armed = 1;
			armTime = now_ms;
		}
	}
	else
	{
		if ((armed && (uint32_t)(now_ms - armTime) >= FALL_FREE_MS &&
			 mag > FALL_IMPACT_LSB) ||
			(LastStrongImpactMs != 0 &&
			 (uint32_t)(now_ms - LastStrongImpactMs) < FALL_IMPACT_WINDOW_MS))
		{
			postImpact = 1;
			postImpactTime = now_ms;
			maxDev = 0;
			armed = 0;
			return;
		}
		armed = 0;
	}

	if (postImpact)
	{
		int32_t dev = (int32_t)fabsf(mag - GRAVITY_LSB);
		if (dev > maxDev)
		{
			maxDev = dev;
		}

		if ((uint32_t)(now_ms - postImpactTime) >= FALL_STILL_MS)
		{
			if (maxDev < FALL_STILL_LSB)
			{
				FallAlarm = 1;
				FallAlarmTime = now_ms;
			}
			postImpact = 0;
		}
	}
}

/**
  * @brief 喂入一个加速度计采样, 更新抬腕/跌倒检测与姿态角.
  * @param  ax, ay, az  MPU6050 原始 16 位加速度数据
  * @param  now_ms      当前时间 ms
  */
void Motion_Update(int16_t ax, int16_t ay, int16_t az, uint32_t now_ms)
{
	float axf = (float)ax;
	float ayf = (float)ay;
	float azf = (float)az;
	float pitch = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * 180.0f / PI_F;
	float roll  = atan2f(ayf, azf) * 180.0f / PI_F;
	float mag   = sqrtf(axf * axf + ayf * ayf + azf * azf);

	/* 处理MPU6050硬件中断: 读取INT_STATUS确认来源并自动清除 */
	if (MotionIntFlag)
	{
		uint8_t st;

		MotionIntFlag = 0;
		st = MPU6050_ReadIntStatus();
		if (st & 0x40) { HwFreeFallMs = now_ms; }
	}

	PitchDeg10 = (int16_t)(pitch * 10.0f);
	RollDeg10  = (int16_t)(roll * 10.0f);

	/* 运动检测: 相邻两帧三轴加速度变化量超过阈值则刷新"最近运动"时间戳 */
	if (HavePrevSample)
	{
		int32_t delta = 0;

		if (ax > PrevAx) { delta += (int32_t)(ax - PrevAx); }
		else             { delta += (int32_t)(PrevAx - ax); }
		if (ay > PrevAy) { delta += (int32_t)(ay - PrevAy); }
		else             { delta += (int32_t)(PrevAy - ay); }
		if (az > PrevAz) { delta += (int32_t)(az - PrevAz); }
		else             { delta += (int32_t)(PrevAz - az); }

		if (delta > MOTION_DELTA_LSB)
		{
			LastMotionMs = now_ms;
		}
		if (delta > FALL_IMPACT_DELTA_LSB)
		{
			LastStrongImpactMs = now_ms;
		}
	}
	PrevAx = ax;
	PrevAy = ay;
	PrevAz = az;
	HavePrevSample = 1;

	Wrist_Update(pitch, now_ms);
	Fall_Update(mag, now_ms);
}

uint8_t Motion_IsMotionRecent(uint32_t windowMs)
{
	uint32_t now = (uint32_t)xTaskGetTickCount();

	if (LastMotionMs == 0)
	{
		return 0;       /* 从未检测到运动 */
	}
	return ((uint32_t)(now - LastMotionMs) < windowMs);
}

uint8_t Motion_IsRaised(void)
{
	return Raised;
}

uint8_t Motion_IsFall(void)
{
	return FallAlarm;
}

uint8_t Motion_IsHwFreeFallRecent(uint32_t windowMs)
{
	uint32_t now = (uint32_t)xTaskGetTickCount();

	if (HwFreeFallMs == 0)
	{
		return 0;
	}
	return ((uint32_t)(now - HwFreeFallMs) < windowMs);
}

int16_t Motion_GetPitch(void)
{
	return PitchDeg10;
}

int16_t Motion_GetRoll(void)
{
	return RollDeg10;
}
