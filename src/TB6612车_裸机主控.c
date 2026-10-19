/* ============================================================
 * PS2 手柄控制四轮麦轮小车 —— STM32F103C8T6 裸机程序（寄存器版）
 *
 * 手柄：塔克创新 PS2 无线手柄（模拟模式）
 * 驱动：2 个 TB6612 + 4 个 TT 马达（麦克纳姆轮，X 型安装）
 *
 * 接线：
 *   —— PS2 ——
 *   DATA -> PA4，CMD -> PA5，CS -> PA6，CLK -> PA7
 *   —— TB6612 #1（左前 LF / 右前 RF）——
 *   PWMA -> PA0 (TIM2_CH1)    AIN1 -> PB0   AIN2 -> PB1   (LF)
 *   PWMB -> PA1 (TIM2_CH2)    BIN1 -> PB5   BIN2 -> PB6   (RF)
 *   STBY -> PB12
 *   —— TB6612 #2（左后 LB / 右后 RB）——
 *   PWMA -> PA2 (TIM2_CH3)    AIN1 -> PB7   AIN2 -> PB8   (LB)
 *   PWMB -> PA3 (TIM2_CH4)    BIN1 -> PB9   BIN2 -> PB10  (RB)
 *   STBY -> PB11
 *   —— 公共 ——
 *   两个 TB6612 的 VCC -> 3.3V/5V（逻辑），VM -> 电池正极（勿接 STM32）
 *   电池负极、TB6612 GND、STM32 GND 三处【共地】
 *
 * 控制（双摇杆全向）：
 *   左摇杆前后 -> 前进/后退
 *   左摇杆左右 -> 左移/右移
 *   右摇杆左右 -> 原地左转/右转
 * 安全：手柄无信号自动停全部马达
 * LED：慢闪=无信号，常亮=有信号无按键，快闪=有按键
 * ============================================================ */

#include <stdint.h>

/* ================= 寄存器定义 ================= */
#define RCC_BASE          0x40021000UL
#define RCC_APB2ENR       (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_APB1ENR       (*(volatile uint32_t *)(RCC_BASE + 0x1C))
#define RCC_APB2ENR_IOPAEN  (1UL << 2)
#define RCC_APB2ENR_IOPBEN  (1UL << 3)
#define RCC_APB2ENR_IOPCEN  (1UL << 4)
#define RCC_APB1ENR_TIM2EN  (1UL << 0)

typedef struct {
    volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2,
                      CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3,
                      CCR4, BDTR, DCR, DMAR;
} TIM_TypeDef;

#define GPIOA   ((GPIO_TypeDef *)0x40010800UL)
#define GPIOB   ((GPIO_TypeDef *)0x40010C00UL)
#define GPIOC   ((GPIO_TypeDef *)0x40011000UL)
#define TIM2    ((TIM_TypeDef  *)0x40000000UL)

/* ================= PS2 / LED 引脚操作 ================= */
#define PS2_DAT_READ()  ((GPIOA->IDR >> 4) & 1UL)
#define PS2_CMD_HIGH()  (GPIOA->ODR |=  (1UL << 5))
#define PS2_CMD_LOW()   (GPIOA->ODR &= ~(1UL << 5))
#define PS2_SEL_HIGH()  (GPIOA->ODR |=  (1UL << 6))
#define PS2_SEL_LOW()   (GPIOA->ODR &= ~(1UL << 6))
#define PS2_CLK_HIGH()  (GPIOA->ODR |=  (1UL << 7))
#define PS2_CLK_LOW()   (GPIOA->ODR &= ~(1UL << 7))

#define LED_ON()        (GPIOC->ODR &= ~(1UL << 13))
#define LED_OFF()       (GPIOC->ODR |=  (1UL << 13))
#define LED_TOGGLE()    (GPIOC->ODR ^=  (1UL << 13))

/* ================= 电机定义 ================= */
typedef struct {
    uint16_t in1, in2;                 /* 方向脚位掩码 */
    volatile uint32_t *ccr;            /* TIM2->CCRx */
} Motor_Def;

/* 索引 0=LF 1=RF 2=LB 3=RB */
static const Motor_Def g_motor[4] = {
    { (1UL << 0),  (1UL << 1),  &TIM2->CCR1 },   /* LF: PB0/PB1, PA0 */
    { (1UL << 5),  (1UL << 6),  &TIM2->CCR2 },   /* RF: PB5/PB6, PA1 */
    { (1UL << 7),  (1UL << 8),  &TIM2->CCR3 },   /* LB: PB7/PB8, PA2 */
    { (1UL << 9),  (1UL << 10), &TIM2->CCR4 },   /* RB: PB9/PB10, PA3 */
};

/* ================= 电机速度标定系数 =================
 * 四个 TT 电机个体有差异，用系数把各轮速度对齐（防止跑偏）。
 * 系数 = 该轮目标转速 / 实际转速，范围约 0.70 ~ 1.30。
 * 调法：以「跑得最快的轮子」为基准(1000)，其余偏慢的轮子系数调大。
 * 单位：千分之几（1000 = 不修正）。
 *
 * 当前：以左前(LF)为基准，右前/左后/右后假设略慢，先各+20% 试。
 * 如果实际是某个轮子特别慢，按标定方法单独调该轮的系数。 */
static const int32_t g_cal[4] = {
    1000,  /* LF 左前：基准（最快） */
    1200,  /* RF 右前：+20% */
    1200,  /* LB 左后：+20% */
    1200,  /* RB 右后：+20% */
};

#define STBY1_HIGH()    (GPIOB->ODR |=  (1UL << 12))
#define STBY1_LOW()     (GPIOB->ODR &= ~(1UL << 12))
#define STBY2_HIGH()    (GPIOB->ODR |=  (1UL << 11))
#define STBY2_LOW()     (GPIOB->ODR &= ~(1UL << 11))

/* 调试用全局变量 */
volatile uint8_t  g_raw[9] = {0};
volatile uint8_t  g_mode = 0;
volatile uint16_t g_buttons = 0;

/* ============ 按键位定义（按下=1，已取反） ============ */
#define PSB_SELECT    0x0001
#define PSB_L3        0x0002
#define PSB_R3        0x0004
#define PSB_START     0x0008
#define PSB_UP        0x0010
#define PSB_RIGHT     0x0020
#define PSB_DOWN      0x0040
#define PSB_LEFT      0x0080
#define PSB_L2        0x0100
#define PSB_R2        0x0200
#define PSB_L1        0x0400
#define PSB_R1        0x0800
#define PSB_TRIANGLE  0x1000
#define PSB_CIRCLE    0x2000
#define PSB_CROSS     0x4000
#define PSB_SQUARE    0x8000

/* ================= 延时 ================= */
static void delay_us(uint32_t us)
{
    volatile uint32_t n = us * 3;
    while (n) n--;
}

static void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}

/* ================= 系统初始化 ================= */
void SystemInit(void) { }
void __libc_init_array(void) { }

/* ================= GPIO 初始化 ================= */
static void gpio_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    /* PA4 输入上拉（DAT），PA5/6/7 输出（CMD/CS/CLK）；PA0~PA3 留给 PWM */
    GPIOA->CRL &= 0x0000FFFFUL;
    GPIOA->CRL |= (0x8UL << 16) | (0x3UL << 20) | (0x3UL << 24) | (0x3UL << 28);
    GPIOA->ODR |= (1UL << 4);

    /* 电机方向脚：PB0/1/5/6/7（CRL）+ PB8/9/10（CRH）输出推挽 50MHz */
    GPIOB->CRL = (GPIOB->CRL & 0x000FFF00UL)
               | (0x3UL << 0) | (0x3UL << 4)
               | (0x3UL << 20) | (0x3UL << 24) | (0x3UL << 28);
    GPIOB->CRH = (GPIOB->CRH & 0xFFF00000UL)
               | (0x3UL << 0) | (0x3UL << 4) | (0x3UL << 8)
               | (0x3UL << 12) | (0x3UL << 16);   /* PB11/PB12 = STBY */

    /* 方向脚 + STBY 初始全低（停马达、禁用驱动） */
    GPIOB->ODR &= ~((1UL << 0) | (1UL << 1) | (1UL << 5) | (1UL << 6)
                  | (1UL << 7) | (1UL << 8) | (1UL << 9) | (1UL << 10)
                  | (1UL << 11) | (1UL << 12));

    /* PC13 输出推挽 2MHz，初始高（LED 灭） */
    GPIOC->CRH &= ~(0xFUL << 20);
    GPIOC->CRH |=  (0x2UL << 20);
    GPIOC->ODR |=  (1UL << 13);

    PS2_SEL_HIGH();
    PS2_CLK_HIGH();
    PS2_CMD_HIGH();
}

/* ================= TIM2 四路 PWM 初始化（1kHz） ================= */
static void tim2_pwm_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* PA0~PA3 = 复用推挽输出（TIM2_CH1~CH4） */
    GPIOA->CRL &= 0xFFFF0000UL;
    GPIOA->CRL |= (0xBUL << 0) | (0xBUL << 4) | (0xBUL << 8) | (0xBUL << 12);

    TIM2->PSC   = 0;                              /* 8MHz/(0+1) = 8MHz */
    TIM2->ARR   = 799;                            /* 8MHz/800 = 10kHz PWM（提升扭矩） */
    TIM2->CCMR1 = (6UL << 4)  | (1UL << 3)        /* CH1 PWM1 */
                | (6UL << 12) | (1UL << 11);      /* CH2 PWM1 */
    TIM2->CCMR2 = (6UL << 4)  | (1UL << 3)        /* CH3 PWM1 */
                | (6UL << 12) | (1UL << 11);      /* CH4 PWM1 */
    TIM2->CCER  = (1UL << 0) | (1UL << 4) | (1UL << 8) | (1UL << 12);
    TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM2->CCR4 = 0;
    TIM2->CR1   = (1UL << 7) | (1UL << 0);        /* ARPE + CEN */
}

/* ================= 电机控制 ================= */
/* speed: -999~+999，正=正转，负=反转，0=停 */
static void motor_set(uint8_t m, int16_t speed)
{
    /* 先乘标定系数，再取绝对值限幅 */
    int32_t s = (int32_t)speed * g_cal[m] / 1000;
    if (s >  799) s =  799;
    if (s < -799) s = -799;

    uint16_t mag = (s < 0) ? (uint16_t)(-s) : (uint16_t)s;

    if (s > 0)
    {
        GPIOB->BSRR = g_motor[m].in1;             /* IN1=1 */
        GPIOB->BRR  = g_motor[m].in2;             /* IN2=0 */
    }
    else if (s < 0)
    {
        GPIOB->BRR  = g_motor[m].in1;             /* IN1=0 */
        GPIOB->BSRR = g_motor[m].in2;             /* IN2=1 */
    }
    else
    {
        GPIOB->BRR  = g_motor[m].in1 | g_motor[m].in2;   /* 都 0 */
    }
    *g_motor[m].ccr = mag;
}

static void all_motor_stop(void)
{
    motor_set(0, 0); motor_set(1, 0); motor_set(2, 0); motor_set(3, 0);
}

/* 速度缩放：-128~127 -> -799~799，并加「最低起步电压」避免低速无力 */
#define PWM_MAX      799
#define PWM_MIN      200     /* 起步最低占空比，低于此直接置 0，防止抖动无力 */
static int16_t scale_speed(int16_t v)
{
    if (v == 0) return 0;

    int32_t r = (int32_t)v * PWM_MAX / 128;
    if (r >  PWM_MAX) r =  PWM_MAX;
    if (r < -PWM_MAX) r = -PWM_MAX;

    /* 低于最低起步值则抬升到 PWM_MIN，保证起步有足够扭矩 */
    if (r > 0 && r <  PWM_MIN) r =  PWM_MIN;
    if (r < 0 && r > -PWM_MIN) r = -PWM_MIN;

    return (int16_t)r;
}

/* ================= 麦轮运动学（X 型安装） ================= */
/* vx=前后(+前)  vy=左右(+右)  wz=旋转(+左转)，范围约 -128~127 */
static void mecanum_move(int16_t vx, int16_t vy, int16_t wz)
{
    int16_t lf = scale_speed(vx + vy + wz);
    int16_t rf = scale_speed(vx - vy - wz);
    int16_t lb = scale_speed(vx - vy + wz);
    int16_t rb = scale_speed(vx + vy - wz);

    motor_set(0, lf);
    motor_set(1, rf);
    motor_set(2, lb);
    motor_set(3, rb);
}

/* ================= PS2 通信 ================= */
static uint8_t ps2_shift(uint8_t cmd)
{
    uint8_t in = 0;
    for (int i = 0; i < 8; i++)
    {
        if (cmd & (1u << i)) PS2_CMD_HIGH(); else PS2_CMD_LOW();

        PS2_CLK_LOW();
        delay_us(2);
        if (PS2_DAT_READ()) in |= (1u << i);
        PS2_CLK_HIGH();
        delay_us(2);
    }
    PS2_CMD_HIGH();
    delay_us(10);
    return in;
}

static void ps2_transfer(const uint8_t *out, uint8_t *in, uint8_t len)
{
    PS2_SEL_LOW();
    delay_us(20);
    for (uint8_t i = 0; i < len; i++)
        in[i] = ps2_shift(out[i]);
    PS2_SEL_HIGH();
    delay_us(20);
}

static uint8_t ps2_read_raw(uint8_t raw[9])
{
    static const uint8_t cmd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    ps2_transfer(cmd, raw, 9);
    return raw[1];
}

/* 配置手柄进入模拟模式（红/绿/红绿灯），让摇杆数据有效 */
static uint8_t ps2_config(void)
{
    uint8_t resp[9] = {0};

    for (int t = 0; t < 5; t++)
    {
        const uint8_t enter[] = {0x01, 0x43, 0x00, 0x01, 0x00};
        ps2_transfer(enter, resp, 5);
        delay_ms(2);

        const uint8_t lock[]  = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
        ps2_transfer(lock, resp, 9);
        delay_ms(2);

        const uint8_t exit[]  = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
        ps2_transfer(exit, resp, 9);
        delay_ms(2);

        uint8_t m = ps2_read_raw(resp);
        if (m == 0x73 || m == 0x53) return 1;

        delay_ms(50);
    }
    return 0;
}

/* ================= 主函数 ================= */
int main(void)
{
    gpio_init();
    tim2_pwm_init();

    for (int i = 0; i < 6; i++)        /* 开机自检：快闪 3 下 */
    {
        LED_TOGGLE();
        delay_ms(100);
    }
    LED_OFF();

    ps2_config();                      /* 上电主动切一次模拟模式 */

    uint32_t cfg_tick = 0;

    while (1)
    {
        g_mode = ps2_read_raw((uint8_t *)g_raw);

        if (g_mode == 0xFF)            /* 无信号 -> 停全部 + STBY 禁用 */
        {
            all_motor_stop();
            STBY1_LOW();
            STBY2_LOW();
            LED_TOGGLE();
            delay_ms(150);
            continue;
        }

        STBY1_HIGH();
        STBY2_HIGH();
        g_buttons = (uint16_t)(((uint8_t)~g_raw[4] << 8) | (uint8_t)~g_raw[3]);

        if (g_mode == 0x41)            /* 数字模式：停 + 周期性切模拟 */
        {
            all_motor_stop();
            if (cfg_tick >= 200)
            {
                cfg_tick = 0;
                ps2_config();
            }
            else
            {
                cfg_tick++;
            }
            LED_ON();
            delay_ms(10);
            continue;
        }

        /* 只有明确模拟模式才控电机，其余值一律停 */
        if (g_mode != 0x73 && g_mode != 0x53 && g_mode != 0x79)
        {
            all_motor_stop();
            LED_ON();
            delay_ms(10);
            continue;
        }

        /* 模拟模式：双摇杆全向控制 */
        cfg_tick = 0;
        int16_t ly = 0x80 - (int16_t)g_raw[8];     /* 左摇杆Y：上推=+前 */
        int16_t lx = (int16_t)g_raw[7] - 0x80;     /* 左摇杆X：右推=+右 */
        int16_t rx = (int16_t)g_raw[5] - 0x80;     /* 右摇杆X：右推=+右转 */

        if (ly > -12 && ly < 12) ly = 0;           /* 死区 */
        if (lx > -12 && lx < 12) lx = 0;
        if (rx > -12 && rx < 12) rx = 0;

        mecanum_move(ly, lx, rx);

        if (g_buttons || ly || lx || rx)   /* 有按键或摇杆动作 -> 快闪 */
        {
            LED_TOGGLE();
            delay_ms(40);
        }
        else
        {
            LED_ON();
            delay_ms(10);
        }
    }
    return 0;
}
