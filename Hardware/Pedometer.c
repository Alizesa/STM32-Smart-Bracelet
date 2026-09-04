/**
  * Pedometer (step counter) - processes MPU6050 accelerometer data.
  *
  * Algorithm: compute the |x|+|y|+|z| acceleration magnitude, low-pass
  * filter it, track a slow drifting baseline (resting magnitude) and
  * count a step each time the filtered magnitude crosses the high
  * threshold and then drops back below the low threshold. A minimum
  * step interval rejects double counts / vibration.
  */
#include "stm32f10x.h"
#include <stdlib.h>
#include "Pedometer.h"

/* thresholds in |ax|+|ay|+|az| LSB (sensor configured for +/-16g) */
#define PEDO_HIGH_THRESHOLD     450
#define PEDO_LOW_THRESHOLD      150
#define PEDO_MIN_INTERVAL_MS    300

static int32_t  MagFiltered;
static int32_t  Baseline;
static uint8_t  Crossing;
static uint32_t StepCount;
static uint32_t LastStepMs;

void Pedometer_Init(void)
{
	MagFiltered = 0;
	Baseline = 0;
	Crossing = 0;
	StepCount = 0;
	LastStepMs = 0;
}

/**
  * @brief  Feed one accelerometer sample into the step detector.
  * @param  ax, ay, az  raw 16-bit accelerometer data
  * @param  now_ms      current time in ms (e.g. xTaskGetTickCount())
  */
void Pedometer_Update(int16_t ax, int16_t ay, int16_t az, uint32_t now_ms)
{
	int32_t mag;
	int32_t delta;

	mag = (int32_t)abs(ax) + (int32_t)abs(ay) + (int32_t)abs(az);

	/* low-pass filter (smooth the magnitude) */
	if (MagFiltered == 0)
	{
		MagFiltered = mag;
	}
	else
	{
		MagFiltered += (mag - MagFiltered) >> 3;
	}

	/* track the resting baseline (slowly following the minimum) */
	if (MagFiltered < Baseline)
	{
		Baseline = MagFiltered;
	}
	else if ((MagFiltered - Baseline) > 100)
	{
		Baseline += (MagFiltered - Baseline - 100) >> 6;
	}

	delta = MagFiltered - Baseline;

	/* rising edge above high threshold */
	if ((delta > PEDO_HIGH_THRESHOLD) && !Crossing)
	{
		Crossing = 1;
	}

	/* falling edge below low threshold => one step */
	if ((delta < PEDO_LOW_THRESHOLD) && Crossing)
	{
		Crossing = 0;
		if ((now_ms - LastStepMs) >= PEDO_MIN_INTERVAL_MS)
		{
			StepCount++;
			LastStepMs = now_ms;
		}
	}
}

uint32_t Pedometer_GetSteps(void) 
{
	return StepCount;
}

void Pedometer_Reset(void)
{
	StepCount = 0;
}
