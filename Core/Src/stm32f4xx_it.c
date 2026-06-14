/**
 * @file  stm32f4xx_it.c
 * @brief IRQ Handlers 
 */
#include "haptic_config.h"
#include "stm32f4xx_hal.h"

/* ── Extern handles  ──────── */
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim10;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_adc;
extern DMA_HandleTypeDef  hdma_uart_tx;

/* ── Core handlers ─────────────────── */
void NMI_Handler(void)         { while(1){} }
void HardFault_Handler(void)   { while(1){} }
void MemManage_Handler(void)   { while(1){} }
void BusFault_Handler(void)    { while(1){} }
void UsageFault_Handler(void)  { while(1){} }
void SVC_Handler(void)         {}
void DebugMon_Handler(void)    {}
void PendSV_Handler(void)      {}
void SysTick_Handler(void)     { HAL_IncTick(); HAL_SYSTICK_IRQHandler(); }

/* ── Peripheral IRQ handlers ─────────────────────────────────── */

/* TIM1 + TIM10  */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE))
        HAL_TIM_IRQHandler(&htim10); /* switch on/off T = 10µs*/

    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))
        HAL_TIM_IRQHandler(&htim1); 
}

/* USART2 – RX Unity */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* DMA2 Stream0 – ADC DMA */
void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc);
}

/* DMA1 Stream6 → USART2 TX  */
void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart_tx);
}