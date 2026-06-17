/**
 * @file  haptic_config.h
 */
#ifndef HAPTIC_CONFIG_H
#define HAPTIC_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#define NUM_MOTORS      3
#define COUNTS_PER_REV  8000u   /* 200PPR × 4 × 10 capstan */
#define PWM_ARR         4199u   /* 84MHz/4200 = 20kHz */

/* ACS712-05B */
#define ACS_VREF        2.5f
#define ACS_SENS        0.185f
#define ADC_VREF        3.3f
#define ADC_FS          4096.0f

/* ADC averaging: 100 × 10µs = 1ms per control tick */
#define ADC_N           100u

#define UART_BAUD       115200u
#define RX_SZ           32u   /* "±99999;±99999;±99999\n\0" = 24 bytes max */
#define TX_SZ           96u   /* 9 fields: θ1;θ2;θ3;c1;c2;c3;v0;v1;v2\n, max ~78 bytes */

typedef struct {
    volatile uint16_t adc[ADC_N];
    float  current_mA;
    float  angle_rad;
    float  duty;
    uint8_t dir;
} Motor_t;

typedef enum { SYS_IDLE=0, SYS_RUN=1 } State_t;

extern TIM_HandleTypeDef  htim1, htim2, htim3, htim4, htim5, htim10;
extern ADC_HandleTypeDef  hadc1;
extern DMA_HandleTypeDef  hdma_adc;
extern DMA_HandleTypeDef  hdma_uart_tx;
extern UART_HandleTypeDef huart2;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif
#endif
