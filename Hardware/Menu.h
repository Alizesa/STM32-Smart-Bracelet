#ifndef __MENU_H_
#define __MENU_H_

/**
  * @brief 主菜单及功能模块 (参考 My_SmartWatch_STM32_STL 的界面风格)
  * @details 包含首页时钟、设置页面、图标主菜单以及各功能页面
  */

/**
  * @brief 菜单模块初始化(复位内部状态)
  */
void Menu_Init(void);

/**
  * @brief 菜单任务入口 (FreeRTOS)
  * @param  pvParameters 任务参数(未使用)
  */
void Menu_Task(void *pvParameters);

#endif
