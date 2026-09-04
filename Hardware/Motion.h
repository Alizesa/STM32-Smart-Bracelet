#ifndef __MOTION_H_
#define __MOTION_H_

#include <stdint.h>

void Motion_Init(void);

/**
  * @brief 初始化MPU6050 INT引脚对应的EXTI(抬腕/跌倒硬件中断).
  * @note  需先把MPU6050的INT脚飞线到 PinMap.h 中 MPU6050_INT_* 指定的引脚(默认PB12)
  */
void Motion_IntPinInit(void);

/**
  * @brief EXTI15_10中断服务函数(由 stm32f10x_it.c 调用, 只置标志位)
  */
void Motion_EXTI_IRQHandler(void);

/**
  * @brief 喂入一个加速度计采样, 更新抬腕/跌倒检测与姿态角.
  * @param  ax, ay, az  MPU6050 原始 16 位加速度数据
  * @param  now_ms      当前时间 ms (如 xTaskGetTickCount())
  */
void Motion_Update(int16_t ax, int16_t ay, int16_t az, uint32_t now_ms);

/** @retval 1-已抬腕, 0-未抬腕 */
uint8_t Motion_IsRaised(void);

/** @retval 1-检测到跌倒, 0-正常 */
uint8_t Motion_IsFall(void);

/**
  * @brief 硬件自由落体中断是否在最近 windowMs 内触发过.
  * @note  用于跌倒时立刻亮屏(不等软件1.6s确认)
  */
uint8_t Motion_IsHwFreeFallRecent(uint32_t windowMs);

/**
  * @brief 是否在最近 windowMs 内检测到明显运动(拿起/抬腕等动作).
  * @note  用于"运动亮屏/静止自动息屏", 与当前朝向无关(修正屏幕朝上放平永不熄屏的问题)
  */
uint8_t Motion_IsMotionRecent(uint32_t windowMs);

/** @retval 俯仰角, 单位 0.1度 */
int16_t Motion_GetPitch(void);

/** @retval 横滚角, 单位 0.1度 */
int16_t Motion_GetRoll(void);

#endif
