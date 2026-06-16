/**
 * @file  main.c  –  Haptic Delta 3-DOF, STM32F411CCUx
 *
 * GPIO/CLOCK of peripheral were declared from STM32CubeMX
 * stm32f4xx_hal_msp.c: CubeMX generate.
 * HAL handle - create data struct - start peripheral.
 *
 * Pinout:
 *  PA0  TIM5_CH1  Encoder M1-A    PA8  TIM1_CH1  RPWM M1
 *  PA1  TIM5_CH2  Encoder M1-B    PA9  TIM1_CH2  LPWM M1
 *  PA2  USART2_TX PL2303           PA10 TIM1_CH3  RPWM M2
 *  PA3  USART2_RX PL2303           PA11 TIM1_CH4  LPWM M2
 *  PA4  ADC1_IN4  ACS712 M1       PB0  TIM3_CH3  RPWM M3
 *  PA5  ADC1_IN5  ACS712 M2       PB1  TIM3_CH4  LPWM M3
 *  PA6  ADC1_IN6  ACS712 M3       PB3  TIM2_CH2  Encoder M2-B
 *  PA15 TIM2_CH1  Encoder M2-A    PB6  TIM4_CH1  Encoder M3-A
 *                                  PB7  TIM4_CH2  Encoder M3-B
 */

#include "haptic_config.h"
#include "stm32f4xx_it.h"
#include <stdio.h>
#include <stdlib.h>

/* ════════════════════════════════════════════════════════════════
 *  HAL HANDLES : extern in haptic_config.h + stm32f4xx_it.c
 * ════════════════════════════════════════════════════════════════ */
TIM_HandleTypeDef  htim1, htim2, htim3, htim4, htim5, htim10;
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc;
DMA_HandleTypeDef  hdma_uart_tx;    /* DMA cho UART TX non-blocking */
UART_HandleTypeDef huart2;

/* ════════════════════════════════════════════════════════════════
 *    GLOBAL VARIABLES 
 * ════════════════════════════════════════════════════════════════ */
static Motor_t   motors[NUM_MOTORS];
static State_t   sys_state = SYS_IDLE;
static uint16_t  adc_dma[NUM_MOTORS];   /* DMA write samplel ADc*/

/* ── UART buffers ─────────────────────────────────────────────── */
static uint8_t  rx_byte;
static char     rx_line[RX_SZ];
static uint8_t  rx_idx = 0;
static char     tx_buf[2][TX_SZ];   /* double buffer tránh ghi đè khi DMA đang gửi */
static uint8_t  tx_sel = 0;         /* buffer đang dùng để ghi */

/* ── Ctrl loop ────────────────────────────────────────────────── */
static uint8_t  ctrl_cnt = 0;

/* ── ACS712 calib sensor when start peripheral */
static uint16_t adc_offset[NUM_MOTORS] = {2048u, 2048u, 2048u};

/* ════════════════════════════════════════════════════════════════
 *  PROTOTYPE LOCAL
 * ════════════════════════════════════════════════════════════════ */
static void Clock_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_Init(void);
static void MX_DMA_Init(void);
static void System_Init(void);

static float    adc_to_mA(uint16_t raw, uint16_t offset);
static void     Calibrate_ADC_Offset(void);
static float    filter_avg(volatile uint16_t *buf, uint16_t n);
static int32_t  enc_read(uint8_t m);
static float    enc_to_rad(int32_t cnt);
static float    pi_calc(PI_t *pi, float err, float dt);
static void     motor_set(uint8_t m, float duty, uint8_t dir);
static uint8_t  fmt_f2(char *buf, float v);
static void     uart_send(void);

/* ════════════════════════════════════════════════════════════════
 *  MAIN
 * ════════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    Clock_Init();

    MX_DMA_Init();   /* DMA -> ADC: get sample from ADC*/
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM10_Init();
    MX_USART2_Init();
    System_Init();

    /* start peripheral */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, NUM_MOTORS);
    Calibrate_ADC_Offset();   /* calibrate zero-offset ACS712 */

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* PA8  RPWM M1 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* PA9  LPWM M1 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   /* PA10 RPWM M2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   /* PA11 LPWM M2 */
    HAL_TIM_MspPostInit(&htim1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   /* PB0  RPWM M3 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   /* PB1  LPWM M3 */
    HAL_TIM_MspPostInit(&htim3);

    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);  /* M1 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  /* M2 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  /* M3 */

    /* Reset encoder */
    __HAL_TIM_SET_COUNTER(&htim5, 0u);
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_SET_COUNTER(&htim4, 0u);

    HAL_TIM_Base_Start_IT(&htim10);
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    sys_state = SYS_RUN;
    while (1) {}
}

/* ════════════════════════════════════════════════════════════════
 *  CALLBACK – TIM10 10 micro s 
 *  Callback + tất cả hàm gọi trong cùng file → không cross-TU
 * ════════════════════════════════════════════════════════════════ */
#define UART_DIV    10u     /* gửi UART mỗi 10 vòng PI × 1ms = 10ms → 100 gói/s */
static uint8_t uart_cnt = 0;

/* ── Soft-start: bỏ qua lệnh Unity trong 2s đầu sau SYS_RUN ─── */
#define SOFTSTART_MS    2000u
static uint32_t softstart_cnt = 0u;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM10) return;
    if (sys_state != SYS_RUN)   return;

    /* Soft-start: giữ current_ref = 0 trong SOFTSTART_MS đầu */
    if (softstart_cnt < SOFTSTART_MS) {
        softstart_cnt++;
        for (uint8_t m = 0; m < NUM_MOTORS; m++) {
            motors[m].pi.integ = 0.0f;
            motor_set(m, 0.0f, 0u);
        }
        return;
    }

    /* Lấy mẫu ADC */
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        motors[m].adc[ctrl_cnt] = adc_dma[m];
    if (++ctrl_cnt < ADC_N) return;
    ctrl_cnt = 0;

    /* --- VÒNG LẶP ĐIỀU KHIỂN: 2 TRƯỜNG HỢP --- */
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        // Đọc dòng điện và góc
        motors[m].current_mA = adc_to_mA((uint16_t)filter_avg(motors[m].adc, ADC_N), adc_offset[m]);
        motors[m].angle_rad = enc_to_rad(enc_read(m));

        /* KIỂM TRA 2 TRƯỜNG HỢP (Ý tưởng của bạn) */
        // Dùng một ngưỡng nhỏ (ví dụ 10mA) để lọc sạch các số rác từ Unity
        if (fabsf(motors[m].current_ref_mA) < 5.0f) 
        {
            // TRƯỜNG HỢP 1: KHÔNG VA CHẠM (LÚC THƯỜNG)
            // 1. Xóa sạch bộ nhớ tích phân PI để không bị cộng dồn nhiễu
            motors[m].pi.integ = 0.0f; 
            
            // 2. Ép xuất thẳng PWM = 0 (Động cơ tắt hoàn toàn, thả lỏng)
            motor_set(m, 0.0f, 0u);
        }
        else 
        {
            // TRƯỜNG HỢP 2: CÓ VA CHẠM (DÒNG ĐIỆN LỚN)
            // 1. Tính toán sai số
            float err = motors[m].current_ref_mA - motors[m].current_mA;
            
            // 2. Chạy bộ PI
            float out = pi_calc(&motors[m].pi, err, TS);
            
            // 3. Xuất lực ra động cơ
            motor_set(m, out >= 0.0f ? out : -out, out >= 0.0f ? 0u : 1u);
        }
    }

    /* Gửi dữ liệu UART lên Unity */
    if (++uart_cnt >= UART_DIV) {
        uart_cnt = 0;
        uart_send();
    }
}

/* ════════════════════════════════════════════════════════════════
 *  CALLBACK – Out: Unity - UART - In: STM32
 * ════════════════════════════════════════════════════════════════ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;
        
    uint8_t b = rx_byte;
    
    /* UNITY's Reset comand  */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    
    /* 1. BỎ QUA KÝ TỰ \r NẾU UNITY GỬI WRITELINE (\r\n) */
    if (b == '\r') return;

    if (rx_idx < RX_SZ - 1)
    {
        rx_line[rx_idx++] = (char)b;
    }
    else
    {
        /* Buffer overflow -> reset */
        rx_idx = 0;
        memset(rx_line, 0, RX_SZ);
        return;
    }
    
    if (b != '\n')
        return;
        
    // Chốt chuỗi: Lúc này rx_line sạch sẽ, chỉ chứa dữ liệu, không có \r hay \n
    rx_line[rx_idx] = '\0';

    /* 2. ÉP BUỘC ĐỔI DẤU PHẨY THÀNH DẤU CHẤM */
    // Đề phòng máy tính Windows của bạn đang dùng ngôn ngữ VN (12,5 thay vì 12.5)
    for (uint8_t i = 0; i < rx_idx; i++) {
        if (rx_line[i] == ',') {
            rx_line[i] = '.';
        }
    }

    /* 4. GIẢI MÃ DỮ LIỆU */
    float v0 = 0.0f;
    float v1 = 0.0f;
    float v2 = 0.0f;
    
    // Nếu sscanf giải mã thành công 3 số thực
    if (sscanf(rx_line, "%f;%f;%f", &v0, &v1, &v2) == 3)
    {
        float limit = ICLAMP;
        if (v0 > limit) v0 = limit; else if (v0 < -limit) v0 = -limit;
        if (v1 > limit) v1 = limit; else if (v1 < -limit) v1 = -limit;
        if (v2 > limit) v2 = limit; else if (v2 < -limit) v2 = -limit;

        motors[0].current_ref_mA = v0;
        motors[1].current_ref_mA = v1;
        motors[2].current_ref_mA = v2;
        
        char dbg[64];
        int len = snprintf(dbg, sizeof(dbg), "OK: %.2f %.2f %.2f\r\n", v0, v1, v2);
        HAL_UART_Transmit(&huart2, (uint8_t*)dbg, len, 100);
    }
    else 
    {
        // Báo lỗi nếu Unity gửi sai format
        HAL_UART_Transmit(&huart2, (uint8_t*)"SSCANF_FAIL\r\n", 13, 100);
    }
    
    rx_idx = 0; // Đặt lại index cho gói dữ liệu tiếp theo
}
/* ════════════════════════════════════════════════════════════════
 *  LOCAL FUNC 
 * ════════════════════════════════════════════════════════════════ */

static float adc_to_mA(uint16_t raw, uint16_t offset)
{
    int32_t delta = (int32_t)raw - (int32_t)offset;
    return (float)delta * (ADC_VREF / ADC_FS / ACS_SENS * 1000.0f);
}

static float filter_avg(volatile uint16_t *buf, uint16_t n)
{
    uint32_t s = 0;
    for (uint16_t i = 0; i < n; i++) s += buf[i];
    return (float)s / (float)n;
}

static int32_t enc_read(uint8_t m)
{
    int32_t c;
    if      (m == 0) c = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    else if (m == 1) c = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    else           { c = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
                     if (c > 32767) c -= 65536; }
    return c;
}

static float enc_to_rad(int32_t cnt)
{
    return (float)cnt / (float)COUNTS_PER_REV * (2.0f * (float)M_PI);
}

static float pi_calc(PI_t *pi, float err, float dt)
{
    /* Deadband ±150mA: lọc noise ADC, lực dưới ~1N không cảm nhận được */
    if (fabsf(err) < 150.0f)
    {
        pi->integ = 0.0f;
        return 0.0f;
    }

    pi->integ += err * dt;
    if      (pi->integ >  pi->ilim) pi->integ =  pi->ilim;
    else if (pi->integ < -pi->ilim) pi->integ = -pi->ilim;
    float o = pi->Kp * err + pi->Ki * pi->integ;
    if      (o >  pi->olim) o =  pi->olim;
    else if (o < -pi->olim) o = -pi->olim;
    return o;
}

static void motor_set(uint8_t m, float duty, uint8_t dir)
{
    if (duty > 100.0f) duty = 100.0f;
    uint32_t cmp = (uint32_t)(duty / 100.0f * PWM_ARR);
    switch (m) {
        case 0:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dir==0 ? cmp : 0u);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dir==1 ? cmp : 0u);
            break;
        case 1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dir==0 ? cmp : 0u);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, dir==1 ? cmp : 0u);
            break;
        default:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, dir==0 ? cmp : 0u);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, dir==1 ? cmp : 0u);
            break;
    }
    motors[m].duty = duty;
    motors[m].dir  = dir;
}

static uint8_t fmt_f2(char *buf, float v)
{
    uint8_t n = 0;

    if (v < 0.0f) { buf[n++] = '-'; v = -v; }

    uint32_t ip = (uint32_t)v;
    uint32_t fp = (uint32_t)((v - (float)ip) * 100.0f + 0.5f);
    if (fp >= 100u) { ip++; fp = 0u; }

    char tmp[12]; int8_t ti = 0;
    if (ip == 0u) {
        tmp[ti++] = '0';
    } else {
        uint32_t t = ip;
        while (t) { tmp[ti++] = (char)('0' + t % 10); t /= 10; }
    }
    while (ti > 0) buf[n++] = tmp[--ti];

    buf[n++] = '.';
    buf[n++] = (char)('0' + fp / 10u);
    buf[n++] = (char)('0' + fp % 10u);
    return n;
}

/* ── uart_send_dma: gửi "θ1;θ2;θ3;c1;c2;c3\n" across DMA (non-blocking) ── */
static volatile uint8_t dma_tx_busy = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) dma_tx_busy = 0;
}

static void uart_send(void)
{
    if (dma_tx_busy) return;   /* DMA còn bận, bỏ qua frame này */

    /* Ghi vào buffer không dùng */
    uint8_t buf = tx_sel ^ 1u;
    char *p = tx_buf[buf];

    float d0 = motors[0].angle_rad * (180.0f / (float)M_PI);
    float d1 = motors[1].angle_rad * (180.0f / (float)M_PI);
    float d2 = motors[2].angle_rad * (180.0f / (float)M_PI);
    float c0 = motors[0].current_mA;
    float c1 = motors[1].current_mA;
    float c2 = motors[2].current_mA;

    p += fmt_f2(p, d0); *p++ = ';';
    p += fmt_f2(p, d1); *p++ = ';';
    p += fmt_f2(p, d2); *p++ = ';';
    p += fmt_f2(p, c0); *p++ = ';';
    p += fmt_f2(p, c1); *p++ = ';';
    p += fmt_f2(p, c2); *p++ = '\n';

    uint16_t len = (uint16_t)(p - tx_buf[buf]);
    tx_sel = buf;
    dma_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buf[buf], len) != HAL_OK)
        dma_tx_busy = 0;
}

/* ── System_Init: PI params + reset state ────────────────────── */
static void System_Init(void)
{
    const float Kp[NUM_MOTORS] = { 0.011f, 0.017f, 0.011f  };
    const float Ki[NUM_MOTORS] = { 10.660f, 8.527f, 10.526f };

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        motors[m].pi = (PI_t){ Kp[m], Ki[m], 0.0f, PI_INT_MAX, PI_OUT_MAX };
        motors[m].current_ref_mA = 0.0f;
        motors[m].angle_rad      = 0.0f;
        motors[m].duty           = 0.0f;
        motors[m].dir            = 0u;
        for (uint16_t i = 0; i < ADC_N; i++) motors[m].adc[i] = 2048u;
    }
    ctrl_cnt = 0u;
    rx_idx   = 0u;
}

static void Calibrate_ADC_Offset(void)
{
    HAL_Delay(1000);   /* chờ ADC DMA ổn định */
    uint32_t acc[NUM_MOTORS] = {0, 0, 0};
    for (uint16_t s = 0; s < 1000u; s++) {
        for (uint8_t m = 0; m < NUM_MOTORS; m++)
            acc[m] += adc_dma[m];
        HAL_Delay(1);
    }
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        adc_offset[m] = (uint16_t)(acc[m] / 1000u);
}

/* ════════════════════════════════════════════════════════════════
 *  PERIPHERAL INIT  (CubeMX-style MX_ functions)
 * ════════════════════════════════════════════════════════════════ */

static void Clock_Init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM            = 25;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV4;  /* 84 MHz */
    osc.PLL.PLLQ            = 7;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider       = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider      = RCC_HCLK_DIV2;
    clk.APB2CLKDivider      = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc.Instance                 = DMA2_Stream0;
    hdma_adc.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode                = DMA_CIRCULAR;
    hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_ADC1_Init(void)
{
    hadc1.Instance                    = ADC1;
    hadc1.Init.ClockPrescaler         = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution             = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode           = ENABLE;
    hadc1.Init.ContinuousConvMode     = ENABLE;
    hadc1.Init.DiscontinuousConvMode  = DISABLE;
    hadc1.Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion        = 3;
    hadc1.Init.DMAContinuousRequests  = ENABLE;
    hadc1.Init.EOCSelection           = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ch = { .SamplingTime = ADC_SAMPLETIME_84CYCLES };
    ch.Channel = ADC_CHANNEL_4; ch.Rank = 1; HAL_ADC_ConfigChannel(&hadc1, &ch);
    ch.Channel = ADC_CHANNEL_5; ch.Rank = 2; HAL_ADC_ConfigChannel(&hadc1, &ch);
    ch.Channel = ADC_CHANNEL_6; ch.Rank = 3; HAL_ADC_ConfigChannel(&hadc1, &ch);
}

/* ── Helper: config PWM ────────────────────────────── */
static void _pwm_ch(TIM_HandleTypeDef *h, uint32_t ch)
{
    TIM_OC_InitTypeDef oc = {
        .OCMode     = TIM_OCMODE_PWM1,
        .Pulse      = 0,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE
    };
    HAL_TIM_PWM_ConfigChannel(h, &oc, ch);
}

static void MX_TIM1_Init(void)
{
    /* APB2 = 84 MHz, PSC=0, ARR=4199 → 20 kHz PWM */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = PWM_ARR;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);
    _pwm_ch(&htim1, TIM_CHANNEL_1);
    _pwm_ch(&htim1, TIM_CHANNEL_2);
    _pwm_ch(&htim1, TIM_CHANNEL_3);
    _pwm_ch(&htim1, TIM_CHANNEL_4);
}

static void MX_TIM3_Init(void)
{
    /* APB1 timer clock = 84 MHz */
    htim3.Instance           = TIM3;
    htim3.Init.Prescaler     = 0;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = PWM_ARR;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);
    _pwm_ch(&htim3, TIM_CHANNEL_3);
    _pwm_ch(&htim3, TIM_CHANNEL_4);
}

static void _enc_init(TIM_HandleTypeDef *h, TIM_TypeDef *inst, uint32_t period)
{
    h->Instance          = inst;
    h->Init.Prescaler    = 0;
    h->Init.CounterMode  = TIM_COUNTERMODE_UP;
    h->Init.Period       = period;
    TIM_Encoder_InitTypeDef e = {
        .EncoderMode  = TIM_ENCODERMODE_TI12,
        .IC1Polarity  = TIM_ICPOLARITY_RISING,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1, .IC1Filter = 4,
        .IC2Polarity  = TIM_ICPOLARITY_RISING,
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1, .IC2Filter = 4
    };
    HAL_TIM_Encoder_Init(h, &e);
}

static void MX_TIM2_Init(void) { _enc_init(&htim2, TIM2, 0xFFFFFFFFu); }  /* M2 32-bit */
static void MX_TIM4_Init(void) { _enc_init(&htim4, TIM4, 0xFFFFu); }       /* M3 16-bit */
static void MX_TIM5_Init(void) { _enc_init(&htim5, TIM5, 0xFFFFFFFFu); }  /* M1 32-bit */

static void MX_TIM10_Init(void)
{
    /* APB2 = 84 MHz, PSC=0, ARR=839 → 100 kHz → T = 10 µs */
    htim10.Instance           = TIM10;
    htim10.Init.Prescaler     = 0;
    htim10.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim10.Init.Period        = 839u;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim10);
}

static void MX_USART2_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = UART_BAUD;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* DMA1 Stream6 Ch4 → USART2 TX */
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_uart_tx.Instance                 = DMA1_Stream6;
    hdma_uart_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_uart_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_uart_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart_tx);
    __HAL_LINKDMA(&huart2, hdmatx, hdma_uart_tx);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0);  /* TX DMA - priority thấp hơn */
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);         /* RX - priority cao hơn TX */
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}