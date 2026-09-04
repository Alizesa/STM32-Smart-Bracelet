/**
  * 主菜单及功能模块 (界面风格参考 My_SmartWatch_STM32_STL)
  *
  * 页面结构:
  *   首页时钟(时间/菜单/设置)  ->  图标主菜单  ->  各功能页
  *      |                             |  返回
  *      +-- 设置 -> 日期时间设置
  *
  * 主菜单只保留需要主动操作的功能: 心率(实时波形), 门禁(NFC模拟开门), 数据(计步/温湿度)
  * 抬腕/跌倒为被动功能: 抬腕用于亮屏, 跌倒弹出全屏告警
  * 按键:   KEY1=上一项  KEY2=下一项  KEY3=确认/返回  KEY3长按=快捷返回
  */
#include "stm32f10x.h"
#include "OLED.h"
#include "Key.h"
#include "MyRTC.h"
#include "Motion.h"
#include "Pedometer.h"
#include "FreeRTOS.h"
#include "task.h"

/* ---------------- 由 main.c 各任务产生的共享数据 ---------------- */
extern volatile uint8_t  gHumidity;
extern volatile uint8_t  gTemperature;
extern volatile uint8_t  gDhtError;
extern volatile uint32_t gSteps;
extern volatile uint16_t gHeartRate;
extern volatile uint8_t  gNfcDetected;
extern volatile uint8_t  gNfcUidLen;
extern volatile uint8_t  gNfcUid[8];

#define HR_WAVE_LEN        64
extern volatile int16_t  HR_Wave[HR_WAVE_LEN];
extern volatile uint8_t  HR_WavePos;

/* ---------------- 首页时钟 ---------------- */
#define CLOCK_TIME_X       16
#define CLOCK_TIME_Y       16

/* ---------------- 菜单动画 ---------------- */
#define MENU_ITEM_MAX      4           /* 菜单项数量(仅主动操作的功能) */
#define MENU_FRAME_X       42
#define MENU_FRAME_Y       10
#define MENU_FRAME_W       44
#define MENU_FRAME_H       44
#define MENU_ICON_BASE_X   48
#define MENU_ICON_Y        16
#define MENU_ICON_SIZE     32
#define MENU_ICON_SPACING  48
#define MENU_SLIDE_STEP    4
#define MENU_ENTER_FRAMES  6
#define MENU_ENTER_STEP    8

/* ---------------- 页面枚举 ---------------- */
typedef enum
{
	PAGE_CLOCK = 0,     /* 首页时钟 */
	PAGE_SETTING,       /* 设置 */
	PAGE_MENU,          /* 图标主菜单 */
	PAGE_HEART,         /* 心率 */
	PAGE_NFC,           /* NFC门禁 */
	PAGE_DATA,          /* 数据(计步/温湿度) */
	PAGE_TIMESET        /* 日期时间设置 */
} Page_t;

/* ---------------- 菜单图标表 (索引0=返回, 1-3=主动功能, 4-8=空白填充) ---------------- */
static const uint8_t *const App_Graph[9] =
{
	Menu_Graph[0],      /* 返回 */
	App_Heart,          /* 心率 */
	App_Nfc,            /* 门禁 */
	App_Data,           /* 数据 */
	Menu_Graph[7],      /* 空白填充(抬腕/跌倒为被动功能, 不进菜单) */
	Menu_Graph[7],
	Menu_Graph[7],
	Menu_Graph[7],
	Menu_Graph[7],
};

/* ---------------- 状态变量 ---------------- */
static Page_t CurPage = PAGE_CLOCK;
static uint8_t FrameCount = 0;

static uint8_t ClockSel = 1;        /* 首页光标: 1-菜单 2-设置 */
static uint8_t SettingSel = 1;      /* 设置光标: 1-返回 2-日期时间设置 */

static uint8_t MenuSel = 1;         /* 菜单选中项 1~MENU_ITEM_MAX */
static int16_t AnimOffset = 0;      /* 图标滑动偏移(px) */
static uint8_t MoveState = 0;       /* 1-正在滑动 */
static uint8_t Entering = 0;        /* 1-正在播放进场动画 */
static uint8_t EnterFrame = 0;
static uint8_t EnterTarget = 0;

static uint8_t TimeSel = 1;         /* 时间设置光标 1-返回 2-年 ... 7-秒 */
static uint8_t TimeAdjusting = 0;   /* 1-正在调整当前字段 */
static uint8_t TimeAdjustIdx = 0;   /* 正在调整的字段索引 0-年 ... 5-秒 */

/* 屏幕状态(运动亮屏 / 静止自动息屏, 与朝向无关) */
static uint8_t ScreenOn = 1;
static uint32_t LastActivityMs = 0;
#define SCREEN_AUTO_OFF_MS    10000   /* 无运动且无按键 10s 后自动息屏 */
#define SCREEN_WAKE_MOTION_MS 600     /* 息屏时检测到该窗口内的运动即亮屏 */

/* 跌倒告警弹窗 */
static uint8_t FallAlertActive = 0;

/* ---------------- 小工具函数 ---------------- */

static void Display_HexByte(uint8_t X, uint8_t Y, uint8_t Value)
{
	static const char hex[] = "0123456789ABCDEF";

	OLED_ShowChar(X, Y, hex[(Value >> 4) & 0x0F], OLED_8X16);
	OLED_ShowChar(X + 8, Y, hex[Value & 0x0F], OLED_8X16);
}

static Page_t MenuItem_ToPage(uint8_t item)
{
	switch (item)
	{
		case 1:  return PAGE_CLOCK;     /* 返回 */
		case 2:  return PAGE_HEART;     /* 心率 */
		case 3:  return PAGE_NFC;       /* 门禁 */
		default: return PAGE_DATA;      /* 数据 */
	}
}

/* ---------------- 首页时钟 ---------------- */

static void Draw_Clock(void)
{
	MyRTC_ReadTime();
	OLED_Clear();

	OLED_Printf(0, 0, OLED_6X8, "%d-%d-%d", MyRTC_Time[0], MyRTC_Time[1], MyRTC_Time[2]);
	OLED_Printf(CLOCK_TIME_X, CLOCK_TIME_Y, OLED_12X24, "%02d:%02d:%02d",
			MyRTC_Time[3], MyRTC_Time[4], MyRTC_Time[5]);

	OLED_ShowString(0, 48, "菜单", OLED_8X16);
	OLED_ShowString(96, 48, "设置", OLED_8X16);
	OLED_ShowImage(112, 0, 16, 16, Battery);

	if (ClockSel == 1)
	{
		OLED_ReverseArea(0, 48, 32, 16);
	}
	else
	{
		OLED_ReverseArea(96, 48, 32, 16);
	}
	OLED_Update();
}

static void Clock_Handle(uint8_t key)
{
	if (key == KEY1_SHORT)
	{
		ClockSel--;
		if (ClockSel < 1) { ClockSel = 2; }
	}
	else if (key == KEY2_SHORT)
	{
		ClockSel++;
		if (ClockSel > 2) { ClockSel = 1; }
	}
	else if (key == KEY3_SHORT)
	{
		if (ClockSel == 1)
		{
			CurPage = PAGE_MENU;
			AnimOffset = 0;
			MoveState = 0;
		}
		else
		{
			CurPage = PAGE_SETTING;
			SettingSel = 1;
		}
		OLED_Clear();
		OLED_Update();
	}
	else if (key == KEY3_LONG)
	{
		Pedometer_Reset();      /* 长按清零计步 */
	}
}

/* ---------------- 设置页面 ---------------- */

static void Draw_Setting(void)
{
	OLED_Clear();
	OLED_ShowImage(0, 0, 16, 16, GoBack);
	OLED_ShowString(0, 16, "日期时间设置", OLED_8X16);

	if (SettingSel == 1)
	{
		OLED_ReverseArea(0, 0, 16, 16);
	}
	else
	{
		OLED_ReverseArea(0, 16, 96, 16);
	}
	OLED_Update();
}

static void Setting_Handle(uint8_t key)
{
	if (key == KEY1_SHORT)
	{
		SettingSel--;
		if (SettingSel < 1) { SettingSel = 2; }
	}
	else if (key == KEY2_SHORT)
	{
		SettingSel++;
		if (SettingSel > 2) { SettingSel = 1; }
	}
	else if (key == KEY3_SHORT)
	{
		if (SettingSel == 1)
		{
			CurPage = PAGE_CLOCK;
		}
		else
		{
			CurPage = PAGE_TIMESET;
			TimeSel = 1;
			TimeAdjusting = 0;
		}
		OLED_Clear();
		OLED_Update();
	}
	else if (key == KEY3_LONG)
	{
		CurPage = PAGE_CLOCK;
		OLED_Clear();
		OLED_Update();
	}
}

/* ---------------- 图标主菜单 ---------------- */

static void Menu_Animation(void)
{
	uint8_t i;

	OLED_Clear();
	OLED_ShowImage(MENU_FRAME_X, MENU_FRAME_Y, MENU_FRAME_W, MENU_FRAME_H, Frame);

	for (i = 0; i < MENU_ITEM_MAX; i++)
	{
		int16_t x = (int16_t)MENU_ICON_BASE_X +
		            ((int16_t)i + 1 - (int16_t)MenuSel) * MENU_ICON_SPACING +
		            AnimOffset;
		OLED_ShowImage(x, MENU_ICON_Y, MENU_ICON_SIZE, MENU_ICON_SIZE, App_Graph[i]);
	}
	OLED_Update();
}

static void Menu_SlideStep(void)
{
	if (MoveState == 0) { return; }

	if (AnimOffset < 0)
	{
		AnimOffset += MENU_SLIDE_STEP;
		if (AnimOffset >= 0) { AnimOffset = 0; MoveState = 0; }
	}
	else if (AnimOffset > 0)
	{
		AnimOffset -= MENU_SLIDE_STEP;
		if (AnimOffset <= 0) { AnimOffset = 0; MoveState = 0; }
	}
}

/* 进场动画: 图标整体向下移出屏幕 */
static void MenuEnter_Step(void)
{
	int16_t y = (int16_t)MENU_ICON_Y + (int16_t)EnterFrame * MENU_ENTER_STEP;
	uint8_t sel = MenuSel;

	OLED_Clear();
	if (sel >= 2) { OLED_ShowImage(0, y, 32, 32, App_Graph[sel - 2]); }
	OLED_ShowImage(48, y, 32, 32, App_Graph[sel - 1]);
	if (sel <= MENU_ITEM_MAX - 1) { OLED_ShowImage(96, y, 32, 32, App_Graph[sel]); }
	OLED_Update();

	EnterFrame++;
	if (EnterFrame > MENU_ENTER_FRAMES)
	{
		Entering = 0;
		CurPage = MenuItem_ToPage(EnterTarget);
		OLED_Clear();
		OLED_Update();
	}
}

static void Menu_Handle(uint8_t key)
{
	if (key == KEY1_SHORT)
	{
		/* 上一项(索引更小, 图标在左侧): 目标项从左侧滑入, 故偏移取负 */
		MenuSel--;
		if (MenuSel < 1) { MenuSel = MENU_ITEM_MAX; }
		AnimOffset = -(int16_t)MENU_ICON_SPACING;
		MoveState = 1;
	}
	else if (key == KEY2_SHORT)
	{
		/* 下一项(索引更大, 图标在右侧): 目标项从右侧滑入, 故偏移取正 */
		MenuSel++;
		if (MenuSel > MENU_ITEM_MAX) { MenuSel = 1; }
		AnimOffset = MENU_ICON_SPACING;
		MoveState = 1;
	}
	else if (key == KEY3_SHORT)
	{
		Entering = 1;
		EnterFrame = 0;
		EnterTarget = MenuSel;
		MoveState = 0;
	}
	else if (key == KEY3_LONG)
	{
		CurPage = PAGE_CLOCK;
		OLED_Clear();
		OLED_Update();
	}
}

/* ---------------- 功能页通用: KEY3 返回菜单 ---------------- */

static void SimplePage_Handle(uint8_t key)
{
	if (key == KEY3_SHORT || key == KEY3_LONG)
	{
		CurPage = PAGE_MENU;
		AnimOffset = 0;
		MoveState = 0;
		OLED_Clear();
		OLED_Update();
	}
}

/* ---------------- 抬腕亮屏 / 自动息屏 ---------------- */

static void Screen_Wake(void)
{
	if (!ScreenOn)
	{
		ScreenOn = 1;
		OLED_DisplayOn();
	}
	LastActivityMs = (uint32_t)xTaskGetTickCount();
}

/* These pages represent an unfinished operation rather than an idle screen. */
static uint8_t Screen_PageNeedsAttention(void)
{
	if (CurPage == PAGE_HEART || CurPage == PAGE_NFC)
	{
		return 1;
	}
	return (CurPage == PAGE_TIMESET && TimeAdjusting) ? 1 : 0;
}

static void Screen_CheckTimeout(void)
{
	uint32_t now = (uint32_t)xTaskGetTickCount();

	if (!ScreenOn)
	{
		return;
	}
	/* Do not interrupt measurements, card polling, or an active time edit. */
	if (Screen_PageNeedsAttention())
	{
		LastActivityMs = now;
		return;
	}
	/* 最近有运动, 或刚按过键 → 保持亮屏 */
	if (Motion_IsMotionRecent(SCREEN_AUTO_OFF_MS) ||
		((uint32_t)(now - LastActivityMs) < SCREEN_AUTO_OFF_MS))
	{
		return;
	}
	/* 完全静止且无按键超过阈值 → 息屏(屏幕朝上放平也会熄) */
	ScreenOn = 0;
	OLED_DisplayOff();
}

/* ---------------- 心率页面 ---------------- */

/* 使用固定比例的16x16心形点阵，通过位移表现心跳。 */
static void Draw_HeartShape(int16_t CX, int16_t CY)
{
	int16_t x = CX - 8;
	int16_t y = CY - 8;
	OLED_ShowImage(x, y, 16, 16, Heart_16);
}

static void Draw_Waveform(void)
{
	int16_t v, vmin, vmax, x, y, prevX = -1, prevY = 0;
	uint8_t i;

	vmin = HR_Wave[0];
	vmax = HR_Wave[0];
	for (i = 1; i < HR_WAVE_LEN; i++)
	{
		v = HR_Wave[i];
		if (v < vmin) { vmin = v; }
		if (v > vmax) { vmax = v; }
	}
	if (vmax - vmin < 100) { vmax = vmin + 100; }

	for (i = 0; i < HR_WAVE_LEN; i++)
	{
		uint8_t idx = (uint8_t)((HR_WavePos + i) % HR_WAVE_LEN);
		v = HR_Wave[idx];
		x = (int16_t)(i * 2);
		y = 62 - (int16_t)((int32_t)(v - vmin) * 16 / (vmax - vmin));
		if (prevX >= 0) { OLED_DrawLine(prevX, prevY, x, y); }
		prevX = x;
		prevY = y;
	}
}

static void Draw_Heart(void)
{
	uint8_t beat = (uint8_t)((FrameCount / 4) % 4);
	int16_t heartY = 30;
	if (beat == 1) { heartY = 29; }
	else if (beat == 2) { heartY = 28; }
	else if (beat == 3) { heartY = 29; }

	OLED_Clear();
	OLED_ShowImage(0, 0, 16, 16, GoBack);
	OLED_ShowString(24, 0, "心率", OLED_8X16);
	OLED_ShowString(64, 0, "BPM", OLED_6X8);
	OLED_ShowNum(96, 0, gHeartRate, 3, OLED_8X16);

	Draw_HeartShape(16, heartY);
	Draw_Waveform();
	OLED_Update();
}

/* ---------------- NFC门禁页面 ---------------- */

static void Draw_Nfc(void)
{
	uint8_t i, x;

	OLED_Clear();
	OLED_ShowImage(0, 0, 16, 16, GoBack);
	OLED_ShowString(24, 0, "门禁", OLED_8X16);
	OLED_ShowString(56, 0, "NFC", OLED_6X8);

	if (gNfcDetected)
	{
		OLED_ShowImage(8, 24, 16, 16, Door_16);
		OLED_ShowString(32, 20, "OPEN", OLED_8X16);
		OLED_ShowString(8, 44, "UID:", OLED_6X8);
		x = 40;
		for (i = 0; i < gNfcUidLen && i < 4; i++)
		{
			Display_HexByte(x, 44, gNfcUid[i]);
			x += 16;
		}
	}
	else
	{
		OLED_ShowImage(8, 24, 16, 16, Card_16);
		OLED_ShowString(32, 20, "READY", OLED_8X16);
		OLED_ShowString(8, 44, "PUT CARD", OLED_6X8);
	}
	OLED_Update();
}

/* ---------------- 跌倒全屏告警弹窗 ---------------- */

static void Draw_FallAlert(void)
{
	OLED_Clear();
	OLED_ShowImage(24, 8, 16, 16, Fall_16);
	OLED_ShowString(48, 4, "FALL!", OLED_12X24);
	OLED_ShowString(16, 44, "跌倒!", OLED_8X16);
	OLED_ShowString(72, 44, "PRESS KEY", OLED_6X8);

	if ((FrameCount / 3) % 2)
	{
		OLED_ReverseArea(0, 0, 128, 64);        /* 整屏闪烁告警 */
	}
	OLED_Update();
}

/* ---------------- 数据页面(计步/温湿度) ---------------- */

static void Draw_Data(void)
{
	OLED_Clear();
	OLED_ShowImage(0, 0, 16, 16, GoBack);
	OLED_ShowString(24, 0, "数据", OLED_8X16);

	OLED_ShowImage(8, 20, 16, 16, Steps_16);
	OLED_ShowString(32, 20, "STEPS", OLED_6X8);
	OLED_ShowNum(80, 20, gSteps, 5, OLED_8X16);

	OLED_ShowString(8, 40, "T:", OLED_6X8);
	OLED_ShowNum(24, 40, gTemperature, 2, OLED_6X8);
	OLED_ShowChar(38, 40, 'C', OLED_6X8);
	OLED_ShowString(56, 40, "H:", OLED_6X8);
	OLED_ShowNum(72, 40, gHumidity, 2, OLED_6X8);
	OLED_ShowChar(86, 40, '%', OLED_6X8);
	if (gDhtError)
	{
		OLED_ShowString(96, 40, "E", OLED_6X8);
		OLED_ShowNum(102, 40, gDhtError, 1, OLED_6X8);
	}

	OLED_ShowString(8, 52, "HR:", OLED_6X8);
	OLED_ShowNum(24, 52, gHeartRate, 3, OLED_6X8);
	OLED_Update();
}

/* ---------------- 日期时间设置页面 ---------------- */

static void ChangeRTC_Time(uint8_t idx, uint8_t inc)
{
	int16_t v = MyRTC_Time[idx] + (inc ? 1 : -1);

	switch (idx)
	{
		case 0: if (v < 2000) { v = 2100; } if (v > 2100) { v = 2000; } break;   /* 年 */
		case 1: if (v < 1)    { v = 12;  } if (v > 12)  { v = 1;    } break;     /* 月 */
		case 2: if (v < 1)    { v = 31;  } if (v > 31)  { v = 1;    } break;     /* 日 */
		case 3: if (v < 0)    { v = 23;  } if (v > 23)  { v = 0;    } break;     /* 时 */
		case 4: if (v < 0)    { v = 59;  } if (v > 59)  { v = 0;    } break;     /* 分 */
		default: if (v < 0)   { v = 59;  } if (v > 59)  { v = 0;    } break;     /* 秒 */
	}
	MyRTC_Time[idx] = v;
	MyRTC_SetTime();
}

static void Draw_TimeSet(void)
{
	OLED_Clear();

	if (TimeSel <= 4)
	{
		OLED_ShowImage(0, 0, 16, 16, GoBack);
		OLED_Printf(0, 16, OLED_8X16, "年:%4d", MyRTC_Time[0]);
		OLED_Printf(0, 32, OLED_8X16, "月:%2d", MyRTC_Time[1]);
		OLED_Printf(0, 48, OLED_8X16, "日:%2d", MyRTC_Time[2]);
	}
	else
	{
		OLED_Printf(0, 0, OLED_8X16, "时:%2d", MyRTC_Time[3]);
		OLED_Printf(0, 16, OLED_8X16, "分:%2d", MyRTC_Time[4]);
		OLED_Printf(0, 32, OLED_8X16, "秒:%2d", MyRTC_Time[5]);
	}

	switch (TimeSel)
	{
		case 1: OLED_ReverseArea(0, 0, 16, 16); break;
		case 2: OLED_ReverseArea(24, 16, 32, 16); break;
		case 3: OLED_ReverseArea(24, 32, 16, 16); break;
		case 4: OLED_ReverseArea(24, 48, 16, 16); break;
		case 5: OLED_ReverseArea(24, 0, 16, 16); break;
		case 6: OLED_ReverseArea(24, 16, 16, 16); break;
		case 7: OLED_ReverseArea(24, 32, 16, 16); break;
		default: break;
	}
	OLED_Update();
}

static void TimeSet_Handle(uint8_t key)
{
	if (TimeAdjusting)
	{
		if (key == KEY1_SHORT) { ChangeRTC_Time(TimeAdjustIdx, 1); }
		else if (key == KEY2_SHORT) { ChangeRTC_Time(TimeAdjustIdx, 0); }
		else if (key == KEY3_SHORT) { TimeAdjusting = 0; }
		return;
	}

	if (key == KEY1_SHORT)
	{
		TimeSel--;
		if (TimeSel < 1) { TimeSel = 7; }
	}
	else if (key == KEY2_SHORT)
	{
		TimeSel++;
		if (TimeSel > 7) { TimeSel = 1; }
	}
	else if (key == KEY3_SHORT)
	{
		if (TimeSel == 1)
		{
			CurPage = PAGE_SETTING;
		}
		else
		{
			TimeAdjusting = 1;
			TimeAdjustIdx = (uint8_t)(TimeSel - 2);
		}
		OLED_Clear();
		OLED_Update();
	}
	else if (key == KEY3_LONG)
	{
		CurPage = PAGE_SETTING;
		OLED_Clear();
		OLED_Update();
	}
}

/* ---------------- 任务入口 ---------------- */

void Menu_Init(void)
{
	CurPage = PAGE_CLOCK;
	ClockSel = 1;
	SettingSel = 1;
	MenuSel = 1;
	AnimOffset = 0;
	MoveState = 0;
	Entering = 0;
	TimeSel = 1;
	TimeAdjusting = 0;
	FrameCount = 0;
	ScreenOn = 1;
	LastActivityMs = 0;
	FallAlertActive = 0;
}

void Menu_Task(void *pvParameters)
{
	uint8_t key;
	TickType_t xLastWake = xTaskGetTickCount();

	(void)pvParameters;
	Menu_Init();

	OLED_Clear();
	OLED_Update();

	for (;;)
	{
		/* 动画期间(菜单滑动/进场)用短周期刷新使动画流畅; 平时静态页用40ms省电;
		   跌倒告警用120ms一帧避免闪烁太快。OLED刷一帧约十几ms, 动画以渲染速度为上限。 */
		TickType_t xFrameDelay = pdMS_TO_TICKS(40);
		if (FallAlertActive)
		{
			xFrameDelay = pdMS_TO_TICKS(120);
		}
		else if (CurPage == PAGE_MENU && (MoveState != 0 || Entering != 0))
		{
			xFrameDelay = pdMS_TO_TICKS(8);
		}
		vTaskDelayUntil(&xLastWake, xFrameDelay);
		FrameCount++;

		key = Key_GetEvent();
		if (key)
		{
			Screen_Wake();      /* 按键: 亮屏并重置静止计时 */
		}

		/* 息屏时检测到运动(拿起/抬腕)或自由落体中断 → 亮屏 */
		if (!ScreenOn &&
			(Motion_IsMotionRecent(SCREEN_WAKE_MOTION_MS) ||
			 Motion_IsHwFreeFallRecent(300)))
		{
			Screen_Wake();
		}

		/* 跌倒检测: 亮屏并弹出全屏告警, 按任意键关闭 */
		if (Motion_IsFall())
		{
			Screen_Wake();
			FallAlertActive = 1;
		}
		if (FallAlertActive)
		{
			if (key)
			{
				FallAlertActive = 0;
				OLED_Clear();
				OLED_Update();
			}
			else
			{
				Draw_FallAlert();
			}
			continue;
		}

		/* 亮屏状态下检测静止超时 → 自动息屏 */
		if (ScreenOn)
		{
			Screen_CheckTimeout();
		}
		if (!ScreenOn)
		{
			continue;           /* 已息屏: 本帧不再刷新 */
		}

		/* 播放菜单进场动画期间不响应按键 */
		if (Entering)
		{
			MenuEnter_Step();
			continue;
		}

		switch (CurPage)
		{
			case PAGE_CLOCK:   Clock_Handle(key); break;
			case PAGE_SETTING: Setting_Handle(key); break;
			case PAGE_MENU:    Menu_Handle(key); break;
			case PAGE_HEART:   SimplePage_Handle(key); break;
			case PAGE_NFC:     SimplePage_Handle(key); break;
			case PAGE_DATA:    SimplePage_Handle(key); break;
			case PAGE_TIMESET: TimeSet_Handle(key); break;
			default: CurPage = PAGE_CLOCK; break;
		}

		switch (CurPage)
		{
			case PAGE_CLOCK:   Draw_Clock(); break;
			case PAGE_SETTING: Draw_Setting(); break;
			case PAGE_MENU:
				Menu_SlideStep();
				Menu_Animation();
				break;
			case PAGE_HEART:   Draw_Heart(); break;
			case PAGE_NFC:     Draw_Nfc(); break;
			case PAGE_DATA:    Draw_Data(); break;
			case PAGE_TIMESET: Draw_TimeSet(); break;
			default: break;
		}
	}
}
