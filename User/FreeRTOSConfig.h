/*
 * FreeRTOSConfig.h -- FreeRTOS 内核配置文件
 *
 * 目标平台: STM32F103C8 (Cortex-M3, 4 位中断优先级, 72MHz)
 * 编译器:   Keil MDK ARMCC V5 (RVDS/ARM_CM3 移植层)
 * 内核版本: FreeRTOS Kernel V11.3.0
 *
 * 说明: configUSE_16_BIT_TICKS 已废弃, V11 改用 configTICK_TYPE_WIDTH_IN_BITS。
 *       两者只能定义其中一个, 见 FreeRTOS.h 中的 #error 检查。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* 引入寄存器定义, 用于获取 __NVIC_PRIO_BITS 等信息 */
#include "stm32f10x.h"

/*-----------------------------------------------------------
 * 基础配置
 *-----------------------------------------------------------*/

#define configUSE_PREEMPTION                        1
#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0

#define configCPU_CLOCK_HZ                          ( ( unsigned long ) 72000000 )  /* 系统时钟 72MHz */
#define configTICK_RATE_HZ                          ( ( TickType_t ) 1000 )         /* 系统节拍 1ms */

#define configMAX_PRIORITIES                        ( 5 )                           /* 必须 <= 32 */
#define configMINIMAL_STACK_SIZE                    ( ( unsigned short ) 128 )      /* 空闲任务栈(字) */
#define configMAX_TASK_NAME_LEN                     ( 16 )

#define configTICK_TYPE_WIDTH_IN_BITS               TICK_TYPE_WIDTH_32_BITS         /* 32 位节拍计数 */

#define configTOTAL_HEAP_SIZE                       ( ( size_t ) ( 14 * 1024 ) )    /* heap_4 堆大小 14KB (7 tasks + 队列, 曾因12KB不足导致任务创建失败) */

#define configIDLE_SHOULD_YIELD                     1
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 1
#define configUSE_COUNTING_SEMAPHORES               1
#define configUSE_TASK_NOTIFICATIONS                1
#define configUSE_TRACE_FACILITY                    1
#define configUSE_TIMERS                            0       /* timers.c 已加入工程, 但先关闭定时器守护任务, 需要时置 1 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1

#define configCHECK_FOR_STACK_OVERFLOW              2       /* 触发时调用 vApplicationStackOverflowHook() */
#define configUSE_MALLOC_FAILED_HOOK                1       /* 堆分配失败时调用 vApplicationMallocFailedHook() */

/*-----------------------------------------------------------
 * 内存分配
 *-----------------------------------------------------------*/

#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configSUPPORT_STATIC_ALLOCATION             0

/*-----------------------------------------------------------
 * 中断优先级配置
 *
 * STM32F103 (Cortex-M3) 的 NVIC 使用高 4 位存放优先级 (configPRIO_BITS = 4)。
 * 移植层通过 configMAX_SYSCALL_INTERRUPT_PRIORITY 掩码屏蔽中断:
 *   数字越大优先级越低。内核运行在最低优先级 (0x0f), 可调用 FreeRTOS
 *   API 的中断优先级上限为 5 (即优先级数值 >= 5 的中断才能调用 API)。
 *-----------------------------------------------------------*/

#define configPRIO_BITS                             4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0x0f
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY             ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/*-----------------------------------------------------------
 * 断言: 失败时关闭中断并停在原地, 方便调试
 *-----------------------------------------------------------*/

#define configASSERT( x )                           if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/*-----------------------------------------------------------
 * INCLUDE_ 函数裁剪开关
 *-----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_vTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xTaskGetSchedulerState              1
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_uxTaskGetStackHighWaterMark         1
#define INCLUDE_eTaskGetState                       1
#define INCLUDE_xTimerPendFunctionCall              0

/*-----------------------------------------------------------
 * C 运行时 (newlib) 线程本地存储, 本工程不使用, 保持默认关闭
 *-----------------------------------------------------------*/

#define configUSE_C_RUNTIME_TLS_SUPPORT             0

#endif /* FREERTOS_CONFIG_H */
