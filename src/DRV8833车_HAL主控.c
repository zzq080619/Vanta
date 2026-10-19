/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PS2遥控麦轮小车 v25 完整版
  *                   上电四段校准(每段3秒):
  *                     1. LED常亮    -> 向前推住左摇杆 = 前后轴
  *                     2. LED慢闪    -> 向左推住左摇杆 = 旋转轴(原地转向)
  *                     3. LED快闪    -> 向右推住右摇杆 = 左右平移轴
  *                     4. LED超慢闪  -> 按住START键    = 急停键
  *                   正常运行: 左摇杆=前后+转向 右摇杆=平移 按住START=急停
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdlib.h>

#define PS2_DAT_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define PS2_CMD_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define PS2_CMD_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define PS2_CS_H     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define PS2_CS_L     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define PS2_CLK_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define PS2_CLK_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define PS2_DAT_READ HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)

#define LED_ON       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#define PWM_MAX   1000
#define DEADZONE  12
#define BIT_DLY   4
#define ROT_DIR   (1)    // 转向方向: 左推杆=左轮后转右轮前转(已验证); 若实测相反改成-1
#define REAR_FIX  2      // 后轮接线修正: 0=不修正 1=后轮整体反向 2=后轮左右对调(当前诊断结果)

TIM_HandleTypeDef htim1, htim3, htim4;
uint8_t ps2_data[22];
uint8_t neutral[22];
uint8_t axis_y = 7;      // 前后轴字节
uint8_t axis_x = 6;      // 左右轴字节
uint8_t axis_r = 0;      // 旋转轴字节(0=未校准)
int8_t  sign_y = 1;
int8_t  sign_x = 1;
int8_t  sign_r = 1;
uint8_t estop_byte = 0;  // 急停键所在字节(0=未校准)
uint8_t estop_mask = 0;  // 急停键位掩码

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void Error_Handler(void);

void delay_us(uint32_t us);
void PS2_Cmd(uint8_t cmd);
uint8_t PS2_ReadByte(void);
void PS2_CmdByte(uint8_t cmd);
void PS2_ReadData(void);
void PS2_EnterConfig(void);
void PS2_SetAnalog(void);
void PS2_ExitConfig(void);
void PS2_Calibrate(void);

void Motor_FL(int16_t s);
void Motor_FR(int16_t s);
void Motor_RL(int16_t s);
void Motor_RR(int16_t s);
void Mecanum(int16_t vx, int16_t vy, int16_t w);

/* USER CODE BEGIN 0 */
void delay_us(uint32_t us)
{
    static uint8_t inited = 0;
    if (!inited) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        inited = 1;
    }
    uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < us * (SystemCoreClock / 1000000));
}

void PS2_Cmd(uint8_t cmd)
{
    for (uint8_t i = 0x01; i; i <<= 1) {
        PS2_CLK_H;
        if (cmd & i) PS2_CMD_H; else PS2_CMD_L;
        delay_us(BIT_DLY);
        PS2_CLK_L;
        delay_us(BIT_DLY);
        PS2_CLK_H;
    }
}

uint8_t PS2_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0x01; i; i <<= 1) {
        PS2_CLK_H;
        delay_us(BIT_DLY);
        PS2_CLK_L;
        delay_us(BIT_DLY);
        if (PS2_DAT_READ) byte |= i;
        PS2_CLK_H;
    }
    return byte;
}

void PS2_CmdByte(uint8_t cmd) { PS2_Cmd(cmd); PS2_ReadByte(); }

void PS2_ReadData(void)
{
    PS2_CS_L;
    delay_us(30);
    PS2_Cmd(0x01);
    ps2_data[1] = PS2_ReadByte();
    PS2_Cmd(0x42);
    ps2_data[2] = PS2_ReadByte();
    for (uint8_t i = 3; i <= 21; i++)   // 读满21字节: 兼容手柄的轴/键可能在9字节之后
        ps2_data[i] = PS2_ReadByte();
    PS2_CS_H;
    delay_us(30);
}

void PS2_EnterConfig(void)
{
    PS2_CS_L; delay_us(30);
    PS2_CmdByte(0x01); PS2_CmdByte(0x43); PS2_CmdByte(0x00);
    PS2_CmdByte(0x01); PS2_CmdByte(0x00);
    PS2_CS_H;
    HAL_Delay(10);
}

void PS2_SetAnalog(void)
{
    PS2_CS_L; delay_us(30);
    PS2_CmdByte(0x01); PS2_CmdByte(0x44); PS2_CmdByte(0x00);
    PS2_CmdByte(0x01); PS2_CmdByte(0x03);
    PS2_CmdByte(0x00); PS2_CmdByte(0x00);
    PS2_CmdByte(0x00); PS2_CmdByte(0x00);
    PS2_CS_H;
    HAL_Delay(10);
}

void PS2_ExitConfig(void)
{
    PS2_CS_L; delay_us(30);
    PS2_CmdByte(0x01); PS2_CmdByte(0x43); PS2_CmdByte(0x00);
    PS2_CmdByte(0x00); PS2_CmdByte(0x5A);
    PS2_CmdByte(0x5A); PS2_CmdByte(0x5A);
    PS2_CmdByte(0x5A); PS2_CmdByte(0x5A);
    PS2_CS_H;
    HAL_Delay(10);
}

// 找"偏差最大的字节"(排除已分配的轴)
static int16_t find_best_axis(int32_t *devsum, uint32_t *cnt, uint8_t ex1, uint8_t ex2)
{
    int16_t best = -1; uint32_t bestc = 0;
    for (uint8_t i = 4; i <= 20; i++) {
        if (i == ex1 || i == ex2) continue;
        if (neutral[i] < 0x40) continue;   // 排除无效字节; 0xFF常态的轴也允许参与学习
        if (cnt[i] > bestc) { bestc = cnt[i]; best = i; }
    }
    if (best >= 0 && bestc <= 10) best = -1;
    return best;
}

void PS2_Calibrate(void)
{
    for (uint8_t k = 0; k < 5; k++) { PS2_ReadData(); HAL_Delay(10); }
    for (uint8_t i = 3; i <= 21; i++) neutral[i] = ps2_data[i];

    int32_t  devsum[22] = {0};
    uint32_t cnt[22]    = {0};
    uint32_t t0, last_blink;
    uint8_t  blink_state;
    int16_t  best;

    // ===== 阶段1: LED常亮3秒, 向前推住 -> 前后轴 =====
    t0 = HAL_GetTick();
    LED_ON;
    while (HAL_GetTick() - t0 < 3000) {
        PS2_ReadData();
        for (uint8_t i = 4; i <= 20; i++) {
            int16_t d = (int16_t)ps2_data[i] - (int16_t)neutral[i];
            if (abs(d) > 20) { devsum[i] += d; cnt[i]++; }
        }
        HAL_Delay(5);
    }
    LED_OFF;
    best = find_best_axis(devsum, cnt, 0, 0);
    if (best >= 0) { axis_y = (uint8_t)best; sign_y = (devsum[best] >= 0) ? 1 : -1; }

    // ===== 阶段2: LED慢闪(200ms)3秒, 向左推住左摇杆 -> 旋转轴 =====
    for (uint8_t i = 0; i < 22; i++) { devsum[i] = 0; cnt[i] = 0; }
    t0 = HAL_GetTick(); last_blink = 0; blink_state = 0;
    while (HAL_GetTick() - t0 < 3000) {
        if (HAL_GetTick() - last_blink >= 200) {
            last_blink = HAL_GetTick(); blink_state = !blink_state;
            if (blink_state) LED_ON; else LED_OFF;
        }
        PS2_ReadData();
        for (uint8_t i = 4; i <= 20; i++) {
            int16_t d = (int16_t)ps2_data[i] - (int16_t)neutral[i];
            if (abs(d) > 20) { devsum[i] += d; cnt[i]++; }
        }
        HAL_Delay(5);
    }
    LED_OFF;
    best = find_best_axis(devsum, cnt, axis_y, 0);
    if (best >= 0) { axis_r = (uint8_t)best; sign_r = (devsum[best] >= 0) ? 1 : -1; }   // 左推左摇杆=左转

    // ===== 阶段3: LED快闪(100ms)3秒, 向右推住右摇杆 -> 左右平移轴 =====
    for (uint8_t i = 0; i < 22; i++) { devsum[i] = 0; cnt[i] = 0; }
    t0 = HAL_GetTick(); last_blink = 0; blink_state = 0;
    while (HAL_GetTick() - t0 < 3000) {
        if (HAL_GetTick() - last_blink >= 100) {
            last_blink = HAL_GetTick(); blink_state = !blink_state;
            if (blink_state) LED_ON; else LED_OFF;
        }
        PS2_ReadData();
        for (uint8_t i = 4; i <= 20; i++) {
            int16_t d = (int16_t)ps2_data[i] - (int16_t)neutral[i];
            if (abs(d) > 20) { devsum[i] += d; cnt[i]++; }
        }
        HAL_Delay(5);
    }
    LED_OFF;
    best = find_best_axis(devsum, cnt, axis_y, axis_r);
    if (best >= 0) { axis_x = (uint8_t)best; sign_x = (devsum[best] >= 0) ? 1 : -1; }   // 右推右摇杆=右平移

    // ===== 阶段4: LED超慢闪(500ms)3秒, 按住START键 -> 急停键 =====
    uint32_t bitdiff[22] = {0};
    uint8_t  xormask[22] = {0};
    uint32_t frames = 0;
    t0 = HAL_GetTick(); last_blink = 0; blink_state = 0;
    while (HAL_GetTick() - t0 < 3000) {
        if (HAL_GetTick() - last_blink >= 500) {
            last_blink = HAL_GetTick(); blink_state = !blink_state;
            if (blink_state) LED_ON; else LED_OFF;
        }
        PS2_ReadData();
        frames++;
        for (uint8_t i = 3; i <= 21; i++) {
            if (ps2_data[i] != neutral[i]) { bitdiff[i]++; xormask[i] |= (ps2_data[i] ^ neutral[i]); }
        }
        HAL_Delay(5);
    }
    LED_OFF;
    {
        uint8_t eb = 0; uint32_t ec = 0;
        for (uint8_t i = 3; i <= 21; i++) {
            if (neutral[i] < 0xF0) continue;   // 只认按键字节(常态0xFF)
            if (bitdiff[i] > ec) { ec = bitdiff[i]; eb = i; }
        }
        if (eb && ec > frames / 2 && xormask[eb]) {
            estop_byte = eb;
            estop_mask = xormask[eb] & (~(xormask[eb] - 1));   // 只保留最低差异位
        }
    }
}

void Motor_FL(int16_t s)
{
    if (s > PWM_MAX) s = PWM_MAX;
    if (s < -PWM_MAX) s = -PWM_MAX;
    if (s >= 0) { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, s);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); }
    else        { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -s); }
}
void Motor_FR(int16_t s)
{
    if (s > PWM_MAX) s = PWM_MAX;
    if (s < -PWM_MAX) s = -PWM_MAX;
    if (s >= 0) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s);
                  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0); }
    else        { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -s); }
}
void Motor_RL(int16_t s)
{
    if (s > PWM_MAX) s = PWM_MAX;
    if (s < -PWM_MAX) s = -PWM_MAX;
    if (s >= 0) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, s);
                  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); }
    else        { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
                  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -s); }
}
void Motor_RR(int16_t s)
{
    if (s > PWM_MAX) s = PWM_MAX;
    if (s < -PWM_MAX) s = -PWM_MAX;
    if (s >= 0) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, s);
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); }
    else        { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -s); }
}

void Mecanum(int16_t vx, int16_t vy, int16_t w)
{
    int32_t fl = vx - vy - w;
    int32_t fr = vx + vy + w;
    int32_t rl = vx + vy - w;
    int32_t rr = vx - vy + w;
    int32_t m = abs(fl);
    if (abs(fr) > m) m = abs(fr);
    if (abs(rl) > m) m = abs(rl);
    if (abs(rr) > m) m = abs(rr);
    if (m > PWM_MAX) {
        fl = fl * PWM_MAX / m;
        fr = fr * PWM_MAX / m;
        rl = rl * PWM_MAX / m;
        rr = rr * PWM_MAX / m;
    }
    Motor_FL((int16_t)fl); Motor_FR((int16_t)fr);
#if REAR_FIX == 1
    Motor_RL((int16_t)-rl); Motor_RR((int16_t)-rr);   // 后轮整体反向
#elif REAR_FIX == 2
    Motor_RL((int16_t)rr);  Motor_RR((int16_t)rl);    // 后轮左右对调(物理左后接在右后通道)
#else
    Motor_RL((int16_t)rl);  Motor_RR((int16_t)rr);    // 原始接法
#endif
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    Mecanum(0, 0, 0);

    HAL_Delay(300);
    PS2_EnterConfig();
    PS2_SetAnalog();
    PS2_ExitConfig();

    PS2_Calibrate();

    // 校准完成标记: 快闪3下
    for (uint8_t k = 0; k < 3; k++) {
        LED_ON;  HAL_Delay(150);
        LED_OFF; HAL_Delay(150);
    }

    // ===== 探索模式30秒: 动任何摇杆/按键, LED报"字节号+数值" =====
    // 编码: 长闪(350ms)=十位, 短闪(120ms)=个位, 报字节号; 停1秒后短闪(200ms)报数值高4位
    // 例: 1长闪+2短闪, 然后5短闪 = 字节12, 值0x5x
    // 30秒内电机强制停车; 什么都不动, 30秒后自动进入正常驾驶模式
    {
        uint8_t  last_f[22];
        uint32_t last_rep[22] = {0};
        for (uint8_t i = 0; i < 22; i++) last_f[i] = ps2_data[i];
        uint32_t tE = HAL_GetTick();
        while (HAL_GetTick() - tE < 30000) {
            PS2_ReadData();
            Mecanum(0, 0, 0);
            for (uint8_t i = 3; i <= 21; i++) {
                if (HAL_GetTick() - last_rep[i] < 400) continue;
                if (abs((int16_t)ps2_data[i] - (int16_t)last_f[i]) > 15) {
                    last_rep[i] = HAL_GetTick();
                    last_f[i]   = ps2_data[i];
                    uint8_t ten = i / 10, one = i % 10;
                    for (uint8_t k = 0; k < ten; k++) {
                        LED_ON; HAL_Delay(350); LED_OFF; HAL_Delay(250);
                    }
                    for (uint8_t k = 0; k < one; k++) {
                        LED_ON; HAL_Delay(120); LED_OFF; HAL_Delay(250);
                    }
                    HAL_Delay(1000);
                    uint8_t n = ps2_data[i] >> 4;
                    for (uint8_t k = 0; k < n; k++) {
                        LED_ON; HAL_Delay(200); LED_OFF; HAL_Delay(200);
                    }
                    HAL_Delay(1500);
                }
            }
        }
    }
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN WHILE */
        PS2_ReadData();

        // 急停: 按住START键 -> 强制停车(松开恢复)
        if (estop_mask && ((ps2_data[estop_byte] & estop_mask) == 0)) {
            Mecanum(0, 0, 0);
        } else {
            int16_t ly = (int16_t)(sign_y * ((int16_t)ps2_data[axis_y] - (int16_t)neutral[axis_y]));
            int16_t lx = (int16_t)(sign_x * ((int16_t)ps2_data[axis_x] - (int16_t)neutral[axis_x]));
            int16_t lr = 0;
            if (axis_r)
                lr = (int16_t)(sign_r * ((int16_t)ps2_data[axis_r] - (int16_t)neutral[axis_r]));

            if (abs(ly) < DEADZONE) ly = 0;
            if (abs(lx) < DEADZONE) lx = 0;
            if (abs(lr) < DEADZONE) lr = 0;

            Mecanum((int16_t)(ly * 8),
                    (int16_t)(lx * 8),
                    (int16_t)(ROT_DIR * lr * 8));
        }

        HAL_Delay(10);
        /* USER CODE END WHILE */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    PS2_DAT_H;

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    LED_OFF;
}

static void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfig = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 71;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim1);
}

static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfig = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_2);
}

static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfig = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 71;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim4);
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig, TIM_CHANNEL_4);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
