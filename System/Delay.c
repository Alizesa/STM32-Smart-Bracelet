#include "stm32f10x.h"

/* 直接定义 DWT 和 CoreDebug 相关寄存器地址 */
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000UL)  // DWT 控制寄存器
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004UL)  // DWT 周期计数器
#define DEMCR       (*(volatile uint32_t *)0xE000EDFCUL)  // 调试异常和监视控制寄存器

#define DEMCR_TRCENA   (1UL << 24)   // 使能 DWT 位
#define DWT_CTRL_CYCCNTENA (1UL << 0) // 使能周期计数器位

/* 初始化 DWT，只需调用一次 */
void DWT_Init(void)
{
    DEMCR |= DEMCR_TRCENA;          // 使能 DWT 跟踪单元
    DWT_CYCCNT = 0;                 // 清零计数器
    DWT_CTRL |= DWT_CTRL_CYCCNTENA; // 使能 CYCCNT 计数器
}

/* 微秒级忙等延时 */
void Delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL); // F103 主频 72MHz -> 72 ticks/us

    while ((DWT_CYCCNT - start) < ticks)
    {
        // 忙等
    }
}

void Delay_ms(uint32_t ms)
{
	while (ms--)
    {
        Delay_us(1000);
    }
}
