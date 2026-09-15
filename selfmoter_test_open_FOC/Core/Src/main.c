/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*
 * 程序入口阅读导航
 *
 * 建议按以下顺序阅读本工程：
 * 1. main()：完成 HAL、系统时钟和各外设初始化；
 * 2. FOC 应用初始化：建立电机参数、采样零点、编码器和控制器初始状态；
 * 3. 主循环：处理串口命令、按键和低频状态管理；
 * 4. foc_app.c：根据转矩、速度或位置模式生成本周期的控制目标；
 * 5. foc_motor.c：完成电流采样、坐标变换、电流 PI 和 SVPWM 输出。
 *
 * 高频电流环通常由定时器或 ADC 同步中断驱动，而不是依靠主循环速度。
 * 因此不要在高频控制路径中加入 printf、阻塞延时或复杂计算。
 */
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "bsp.h"
#include "foc_hw.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Privat4e function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Diagnostic_SetNeutralPwm(void);
static void Motor_OpenLoop_StopPwm(void);
static float FOC_Wrap_0_2PI(float x);
static float FOC_Wrap_Pi(float x);
static void Diagnostic_SetSinPwm(float electrical_angle,
                                 float phase_voltage,
                                 float bus_voltage);
static void Diagnostic_ResetStepSample(void);
static void Diagnostic_AddStepSample(float enc_elec_raw, float cmd_angle);
static void Diagnostic_SaveStepResult(uint32_t step, float cmd_angle);
static void Diagnostic_UpdateScalarDebug(void);
static void Diagnostic_ResetResults(void);
static uint32_t Diagnostic_GetPhaseMap(void);
static void Diagnostic_CheckPhaseMapChange(void);
static void Diagnostic_UpdateISR(void);
static uint8_t AS5047P_EvenParity(uint16_t value);
static uint16_t AS5047P_MakeReadCmd(uint16_t addr);
static uint16_t AS5047P_SPI_Transfer16(uint16_t tx_data);
static uint16_t AS5047P_ReadReg(uint16_t addr);
static uint16_t AS5047P_ReadRawAngle(void);
static void AS5047P_ReadDiagnostics(void);
static void AS5047P_ReadDiagnosticsLowRate(void);
static float AS5047P_ReadMechanicalAngleRad(void);

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 电机和诊断测试相关常量. */
#define SFOC_BUS_VOLTAGE              24.0f
#define SFOC_POLE_PAIRS                7.0f
#define SFOC_PWM_FREQ_HZ           20000.0f
#define SFOC_TWO_PI                    6.28318530718f
#define SFOC_PI                        3.14159265359f

/* TIM6 为 20kHz, 诊断循环每 20 个 tick 执行一次, 等效 1kHz. */
#define SFOC_CONTROL_DIVIDER          20U

/* AS5047P 单独测试模式: MOTOR_EN 关闭, TIM1 保持 50% 中点 PWM. */
#define AS5047_ONLY_TEST               0
#define AS5047_POLL_IN_WHILE_TEST      0
#define AS5047_SPI_8BIT_TEST           1
#define AS5047_DIAG_READ_DIVIDER       20U

/* AS5047P 测试得到的方向符号设置. */
#define SFOC_ENCODER_DIR               (-1.0f)
#define MOTOR_SD_DISABLE_LEVEL         GPIO_PIN_RESET

/* 六步相序诊断参数, 用来观察相序和编码器角度对应关系. */
#define DIAG_LOCK_VOLTAGE              0.30f
#define DIAG_RAMP_SEC                  0.10f
#define DIAG_HOLD_SEC                  0.30f
#define DIAG_SAMPLE_LAST_SEC           0.10f
#define DIAG_STEP_COUNT                6U
#define DIAG_STEP_ANGLE_DEG            60.0f
#define DIAG_PHASE_MAP                 0U
#define DIAG_PHASE_MAP_COUNT           6U

/* AS5047P 寄存器地址和数据掩码. */
#define AS5047P_REG_DIAAGC            0x3FFCU
#define AS5047P_REG_MAG               0x3FFDU
#define AS5047P_REG_ANGLEUNC          0x3FFEU
#define AS5047P_REG_ERRFL             0x0001U
#define AS5047P_DATA_MASK             0x3FFFU
#define AS5047P_EF_MASK               0x4000U

/* USER CODE END PD */

/* USER CODE BEGIN PV */

volatile uint32_t g_tim6_debug_count = 0;

volatile uint8_t  g_diag_enable = 0;
volatile uint32_t g_diag_tick = 0;
volatile uint32_t g_diag_div_count = 0;

volatile uint32_t g_diag_step_index = 0;
volatile uint32_t g_diag_cycle_count = 0;
volatile uint32_t g_diag_phase_map = DIAG_PHASE_MAP;
volatile uint8_t  g_diag_ramp_active = 0;
volatile float g_diag_ramp_progress = 0.0f;

volatile float g_diag_cmd_elec_angle = 0.0f;
volatile float g_diag_cmd_elec_angle_deg = 0.0f;

volatile float g_diag_encoder_mech_angle = 0.0f;
volatile float g_diag_encoder_mech_angle_deg = 0.0f;
volatile float g_diag_encoder_elec_angle = 0.0f;
volatile float g_diag_encoder_elec_angle_deg = 0.0f;

volatile float g_diag_current_diff = 0.0f;
volatile float g_diag_current_diff_deg = 0.0f;

volatile float g_debug_ud = 0.0f;
volatile float g_debug_uq = 0.0f;
volatile float g_debug_angle = 0.0f;
volatile float g_debug_rpm = 0.0f;

volatile float g_debug_duty_a = 0.0f;
volatile float g_debug_duty_b = 0.0f;
volatile float g_debug_duty_c = 0.0f;

volatile uint16_t g_as5047_raw = 0;
volatile uint16_t g_as5047_last_rx = 0;
volatile float g_as5047_angle_rad = 0.0f;
volatile float g_as5047_angle_deg = 0.0f;
volatile uint32_t g_as5047_error_count = 0;

volatile uint16_t g_as5047_diaagc_raw = 0;
volatile uint8_t  g_as5047_magl = 0;
volatile uint8_t  g_as5047_magh = 0;
volatile uint8_t  g_as5047_cof = 0;
volatile uint8_t  g_as5047_lf = 0;
volatile uint8_t  g_as5047_agc = 0;
volatile uint16_t g_as5047_mag_value = 0;

volatile uint32_t g_as5047_zero_rx_count = 0;
volatile uint32_t g_as5047_zero_angle_count = 0;
volatile uint32_t g_as5047_zero_diaagc_count = 0;
volatile uint32_t g_as5047_zero_mag_count = 0;

volatile uint16_t g_as5047_angle_raw_frame = 0;
volatile uint16_t g_as5047_diaagc_raw_frame = 0;
volatile uint16_t g_as5047_mag_raw_frame = 0;

/*
 *
 *
 */
volatile uint8_t g_diag_step_valid[DIAG_STEP_COUNT];

volatile float g_diag_step_cmd_deg[DIAG_STEP_COUNT];
volatile float g_diag_step_enc_mech_deg[DIAG_STEP_COUNT];
volatile float g_diag_step_enc_elec_deg[DIAG_STEP_COUNT];
volatile float g_diag_step_diff_deg[DIAG_STEP_COUNT];
volatile float g_diag_step_delta_deg[DIAG_STEP_COUNT];

/*
 *
 */
static float g_sample_sin_sum = 0.0f;
static float g_sample_cos_sum = 0.0f;
static uint32_t g_sample_count = 0;

static uint32_t g_last_step_index = 0xFFFFFFFFU;
static float g_last_cmd_angle = 0.0f;
static uint32_t g_last_phase_map = 0xFFFFFFFFU;

volatile float g_step0_delta_deg = 0.0f;
volatile float g_step1_delta_deg = 0.0f;
volatile float g_step2_delta_deg = 0.0f;
volatile float g_step3_delta_deg = 0.0f;
volatile float g_step4_delta_deg = 0.0f;
volatile float g_step5_delta_deg = 0.0f;

volatile float g_step0_enc_elec_deg = 0.0f;
volatile float g_step1_enc_elec_deg = 0.0f;
volatile float g_step2_enc_elec_deg = 0.0f;
volatile float g_step3_enc_elec_deg = 0.0f;
volatile float g_step4_enc_elec_deg = 0.0f;
volatile float g_step5_enc_elec_deg = 0.0f;

volatile float g_step0_diff_deg = 0.0f;
volatile float g_step1_diff_deg = 0.0f;
volatile float g_step2_diff_deg = 0.0f;
volatile float g_step3_diff_deg = 0.0f;
volatile float g_step4_diff_deg = 0.0f;
volatile float g_step5_diff_deg = 0.0f;

/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void Diagnostic_SetNeutralPwm(void)
{
  uint32_t half;

  half = (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U) / 2U;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, half);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, half);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, half);

  g_debug_duty_a = 0.5f;
  g_debug_duty_b = 0.5f;
  g_debug_duty_c = 0.5f;

  g_debug_ud = 0.0f;
  g_debug_uq = 0.0f;
}

static void Motor_OpenLoop_StopPwm(void)
{
  uint32_t half;

  g_diag_enable = 0;

  half = (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U) / 2U;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, half);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, half);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, half);

  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,
                    MOTOR_EN_Pin,
                    MOTOR_SD_DISABLE_LEVEL);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static float FOC_Wrap_0_2PI(float x)
{
  while (x >= SFOC_TWO_PI)
  {
    x -= SFOC_TWO_PI;
  }

  while (x < 0.0f)
  {
    x += SFOC_TWO_PI;
  }

  return x;
}

static float FOC_Wrap_Pi(float x)
{
  while (x > SFOC_PI)
  {
    x -= SFOC_TWO_PI;
  }

  while (x < -SFOC_PI)
  {
    x += SFOC_TWO_PI;
  }

  return x;
}

/*
 * 诊断用三相正弦 PWM 输出.
 * electrical_angle 是给定电角度, phase_voltage 是相电压幅值.
 */
static void Diagnostic_SetSinPwm(float electrical_angle,
                                 float phase_voltage,
                                 float bus_voltage)
{
  float m;
  uint32_t phase_map;

  float u_base;
  float v_base;
  float w_base;

  float u_out;
  float v_out;
  float w_out;

  float duty_a;
  float duty_b;
  float duty_c;

  uint32_t arr;
  uint32_t ccr_a;
  uint32_t ccr_b;
  uint32_t ccr_c;

  m = 2.0f * phase_voltage / bus_voltage;

  if (m > 0.20f)
  {
    m = 0.20f;
  }

  if (m < 0.0f)
  {
    m = 0.0f;
  }

  u_base = sinf(electrical_angle);
  v_base = sinf(electrical_angle - 2.09439510239f);
  w_base = sinf(electrical_angle + 2.09439510239f);

  phase_map = Diagnostic_GetPhaseMap();

  switch (phase_map)
  {
    case 0U:     /* U V W */
      u_out = u_base;
      v_out = v_base;
      w_out = w_base;
      break;

    case 1U:     /* U W V */
      u_out = u_base;
      v_out = w_base;
      w_out = v_base;
      break;

    case 2U:     /* V U W */
      u_out = v_base;
      v_out = u_base;
      w_out = w_base;
      break;

    case 3U:     /* V W U */
      u_out = v_base;
      v_out = w_base;
      w_out = u_base;
      break;

    case 4U:     /* W U V */
      u_out = w_base;
      v_out = u_base;
      w_out = v_base;
      break;

    default:     /* 5 = W V U */
      u_out = w_base;
      v_out = v_base;
      w_out = u_base;
      break;
  }

  duty_a = 0.5f + 0.5f * m * u_out;
  duty_b = 0.5f + 0.5f * m * v_out;
  duty_c = 0.5f + 0.5f * m * w_out;

  if (duty_a < 0.02f) duty_a = 0.02f;
  if (duty_a > 0.98f) duty_a = 0.98f;

  if (duty_b < 0.02f) duty_b = 0.02f;
  if (duty_b > 0.98f) duty_b = 0.98f;

  if (duty_c < 0.02f) duty_c = 0.02f;
  if (duty_c > 0.98f) duty_c = 0.98f;

  g_debug_duty_a = duty_a;
  g_debug_duty_b = duty_b;
  g_debug_duty_c = duty_c;

  arr = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

  ccr_a = (uint32_t)(duty_a * (float)arr);
  ccr_b = (uint32_t)(duty_b * (float)arr);
  ccr_c = (uint32_t)(duty_c * (float)arr);

  if (ccr_a >= arr) ccr_a = arr - 1U;
  if (ccr_b >= arr) ccr_b = arr - 1U;
  if (ccr_c >= arr) ccr_c = arr - 1U;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

static void Diagnostic_ResetStepSample(void)
{
  g_sample_sin_sum = 0.0f;
  g_sample_cos_sum = 0.0f;
  g_sample_count = 0U;

  g_diag_current_diff = 0.0f;
  g_diag_current_diff_deg = 0.0f;
}

/*
 *
 */
static void Diagnostic_AddStepSample(float enc_elec_raw, float cmd_angle)
{
  float avg;
  float diff;

  g_sample_sin_sum += sinf(enc_elec_raw);
  g_sample_cos_sum += cosf(enc_elec_raw);
  g_sample_count++;

  avg = atan2f(g_sample_sin_sum, g_sample_cos_sum);
  avg = FOC_Wrap_0_2PI(avg);

  diff = FOC_Wrap_Pi(cmd_angle - avg);

  g_diag_current_diff = diff;
  g_diag_current_diff_deg = diff * 180.0f / SFOC_PI;
}

/*
 *
 */
static void Diagnostic_SaveStepResult(uint32_t step, float cmd_angle)
{
  float avg;
  float diff;
  float delta;
  uint32_t prev;

  if (step >= DIAG_STEP_COUNT)
  {
    return;
  }

  if (g_sample_count == 0U)
  {
    return;
  }

  avg = atan2f(g_sample_sin_sum, g_sample_cos_sum);
  avg = FOC_Wrap_0_2PI(avg);

  diff = FOC_Wrap_Pi(cmd_angle - avg);

  g_diag_step_valid[step] = 1U;

  g_diag_step_cmd_deg[step] =
      cmd_angle * 180.0f / SFOC_PI;

  g_diag_step_enc_elec_deg[step] =
      avg * 180.0f / SFOC_PI;

  g_diag_step_enc_mech_deg[step] =
      g_diag_encoder_mech_angle_deg;

  g_diag_step_diff_deg[step] =
      diff * 180.0f / SFOC_PI;

  /*
   *
   *
   */
  if (step == 0U)
  {
    prev = DIAG_STEP_COUNT - 1U;
  }
  else
  {
    prev = step - 1U;
  }

  if (g_diag_step_valid[prev] != 0U)
  {
    delta =
        FOC_Wrap_Pi((g_diag_step_enc_elec_deg[step] -
                     g_diag_step_enc_elec_deg[prev]) *
                    SFOC_PI / 180.0f);

    g_diag_step_delta_deg[step] =
        delta * 180.0f / SFOC_PI;
  }
  else
  {
    g_diag_step_delta_deg[step] = 0.0f;
  }
	Diagnostic_UpdateScalarDebug();
}

static void Diagnostic_UpdateScalarDebug(void)
{
  g_step0_delta_deg = g_diag_step_delta_deg[0];
  g_step1_delta_deg = g_diag_step_delta_deg[1];
  g_step2_delta_deg = g_diag_step_delta_deg[2];
  g_step3_delta_deg = g_diag_step_delta_deg[3];
  g_step4_delta_deg = g_diag_step_delta_deg[4];
  g_step5_delta_deg = g_diag_step_delta_deg[5];

  g_step0_enc_elec_deg = g_diag_step_enc_elec_deg[0];
  g_step1_enc_elec_deg = g_diag_step_enc_elec_deg[1];
  g_step2_enc_elec_deg = g_diag_step_enc_elec_deg[2];
  g_step3_enc_elec_deg = g_diag_step_enc_elec_deg[3];
  g_step4_enc_elec_deg = g_diag_step_enc_elec_deg[4];
  g_step5_enc_elec_deg = g_diag_step_enc_elec_deg[5];

  g_step0_diff_deg = g_diag_step_diff_deg[0];
  g_step1_diff_deg = g_diag_step_diff_deg[1];
  g_step2_diff_deg = g_diag_step_diff_deg[2];
  g_step3_diff_deg = g_diag_step_diff_deg[3];
  g_step4_diff_deg = g_diag_step_diff_deg[4];
  g_step5_diff_deg = g_diag_step_diff_deg[5];
}

static void Diagnostic_ResetResults(void)
{
  uint32_t i;

  g_diag_tick = 0U;
  g_diag_div_count = 0U;
  g_diag_step_index = 0U;
  g_diag_cycle_count = 0U;
  g_diag_ramp_active = 0U;
  g_diag_ramp_progress = 0.0f;

  g_diag_cmd_elec_angle = 0.0f;
  g_diag_cmd_elec_angle_deg = 0.0f;

  g_diag_encoder_mech_angle = 0.0f;
  g_diag_encoder_mech_angle_deg = 0.0f;
  g_diag_encoder_elec_angle = 0.0f;
  g_diag_encoder_elec_angle_deg = 0.0f;

  g_diag_current_diff = 0.0f;
  g_diag_current_diff_deg = 0.0f;

  for (i = 0U; i < DIAG_STEP_COUNT; i++)
  {
    g_diag_step_valid[i] = 0U;
    g_diag_step_cmd_deg[i] = 0.0f;
    g_diag_step_enc_mech_deg[i] = 0.0f;
    g_diag_step_enc_elec_deg[i] = 0.0f;
    g_diag_step_diff_deg[i] = 0.0f;
    g_diag_step_delta_deg[i] = 0.0f;
  }

  Diagnostic_ResetStepSample();
  Diagnostic_UpdateScalarDebug();

  g_last_step_index = 0xFFFFFFFFU;
  g_last_cmd_angle = 0.0f;
}

static uint32_t Diagnostic_GetPhaseMap(void)
{
  uint32_t phase_map;

  phase_map = g_diag_phase_map;

  if (phase_map >= DIAG_PHASE_MAP_COUNT)
  {
    phase_map = 0U;
    g_diag_phase_map = 0U;
  }

  return phase_map;
}

static void Diagnostic_CheckPhaseMapChange(void)
{
  uint32_t phase_map;

  phase_map = Diagnostic_GetPhaseMap();

  if (g_last_phase_map != phase_map)
  {
    g_last_phase_map = phase_map;
    Diagnostic_ResetResults();
  }
}

static void Diagnostic_UpdateISR(void)
{
  float mech_angle;
  float enc_elec_raw;
  float cmd_angle;
  float step_angle_rad;
  float target_angle;
  float target_unwrapped;
  float previous_unwrapped;
  float ramp_progress;
  float diff;

  uint32_t ramp_ticks;
  uint32_t hold_ticks;
  uint32_t step_ticks;
  uint32_t sample_start_tick;
  uint32_t step_index;
  uint32_t tick_in_step;
  uint32_t sample_ticks;
  uint32_t total_step_count;

  if (g_diag_enable == 0U)
  {
    return;
  }

  /*
   *
   */
  g_diag_div_count++;

  if (g_diag_div_count < SFOC_CONTROL_DIVIDER)
  {
    return;
  }

  g_diag_div_count = 0;

  /*
   *
   */
  mech_angle = AS5047P_ReadMechanicalAngleRad();
  AS5047P_ReadDiagnosticsLowRate();

  enc_elec_raw =
      FOC_Wrap_0_2PI(mech_angle *
                     SFOC_POLE_PAIRS *
                     SFOC_ENCODER_DIR);

  if ((AS5047_ONLY_TEST != 0) ||
      (AS5047_POLL_IN_WHILE_TEST != 0))
  {
    Diagnostic_SetNeutralPwm();

    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,
                      MOTOR_EN_Pin,
                      MOTOR_SD_DISABLE_LEVEL);

    g_diag_step_index = 0U;
    g_diag_ramp_active = 0U;
    g_diag_ramp_progress = 0.0f;

    g_diag_cmd_elec_angle = 0.0f;
    g_diag_cmd_elec_angle_deg = 0.0f;

    g_diag_encoder_mech_angle = mech_angle;
    g_diag_encoder_mech_angle_deg =
        mech_angle * 180.0f / SFOC_PI;

    g_diag_encoder_elec_angle = enc_elec_raw;
    g_diag_encoder_elec_angle_deg =
        enc_elec_raw * 180.0f / SFOC_PI;

    g_diag_current_diff = 0.0f;
    g_diag_current_diff_deg = 0.0f;

    g_debug_angle = 0.0f;
    g_debug_ud = 0.0f;
    g_debug_uq = 0.0f;
    g_debug_rpm = 0.0f;

    g_diag_tick++;
    return;
  }

  Diagnostic_CheckPhaseMapChange();

  ramp_ticks =
      (uint32_t)(DIAG_RAMP_SEC *
                 SFOC_PWM_FREQ_HZ /
                 (float)SFOC_CONTROL_DIVIDER);

  hold_ticks =
      (uint32_t)(DIAG_HOLD_SEC *
                 SFOC_PWM_FREQ_HZ /
                 (float)SFOC_CONTROL_DIVIDER);

  if (hold_ticks == 0U)
  {
    hold_ticks = 1U;
  }

  step_ticks =
      ramp_ticks + hold_ticks;

  if (step_ticks == 0U)
  {
    step_ticks = 1U;
  }

  sample_ticks =
      (uint32_t)(DIAG_SAMPLE_LAST_SEC *
                 SFOC_PWM_FREQ_HZ /
                 (float)SFOC_CONTROL_DIVIDER);

  if (sample_ticks == 0U)
  {
    sample_ticks = 1U;
  }

  if (sample_ticks > hold_ticks)
  {
    sample_ticks = hold_ticks;
  }

  sample_start_tick = step_ticks - sample_ticks;

  total_step_count = g_diag_tick / step_ticks;
  step_index = total_step_count % DIAG_STEP_COUNT;
  tick_in_step = g_diag_tick % step_ticks;

  step_angle_rad = DIAG_STEP_ANGLE_DEG * SFOC_PI / 180.0f;

  target_unwrapped = ((float)total_step_count) * step_angle_rad;
  target_angle = FOC_Wrap_0_2PI(target_unwrapped);

  cmd_angle = target_angle;
  g_diag_ramp_active = 0U;
  g_diag_ramp_progress = 1.0f;

  if ((total_step_count > 0U) &&
      (ramp_ticks > 0U) &&
      (tick_in_step < ramp_ticks))
  {
    previous_unwrapped = target_unwrapped - step_angle_rad;
    ramp_progress = ((float)tick_in_step) / (float)ramp_ticks;

    cmd_angle =
        FOC_Wrap_0_2PI(previous_unwrapped +
                      (step_angle_rad * ramp_progress));

    g_diag_ramp_active = 1U;
    g_diag_ramp_progress = ramp_progress;
  }

  /*
   *
   */
  if (g_last_step_index != step_index)
  {
    if (g_last_step_index < DIAG_STEP_COUNT)
    {
      Diagnostic_SaveStepResult(g_last_step_index, g_last_cmd_angle);

      if ((g_last_step_index == (DIAG_STEP_COUNT - 1U)) &&
          (step_index == 0U))
      {
        g_diag_cycle_count++;
      }
    }

    Diagnostic_ResetStepSample();

    g_last_step_index = step_index;
    g_last_cmd_angle = target_angle;
  }

  /*
   *
   */
  Diagnostic_SetSinPwm(cmd_angle,
                       DIAG_LOCK_VOLTAGE,
                       SFOC_BUS_VOLTAGE);
  /*
   *
   */
  diff = FOC_Wrap_Pi(cmd_angle - enc_elec_raw);

  g_diag_step_index = step_index;

  g_diag_cmd_elec_angle = cmd_angle;
  g_diag_cmd_elec_angle_deg =
      cmd_angle * 180.0f / SFOC_PI;

  g_diag_encoder_mech_angle = mech_angle;
  g_diag_encoder_mech_angle_deg =
      mech_angle * 180.0f / SFOC_PI;

  g_diag_encoder_elec_angle = enc_elec_raw;
  g_diag_encoder_elec_angle_deg =
      enc_elec_raw * 180.0f / SFOC_PI;

  g_diag_current_diff = diff;
  g_diag_current_diff_deg =
      diff * 180.0f / SFOC_PI;

  /*
   *
   */
  if (tick_in_step >= sample_start_tick)
  {
    Diagnostic_AddStepSample(enc_elec_raw, target_angle);
  }

  g_debug_angle = cmd_angle;
  g_debug_ud =  0.0f;
  g_debug_uq = 0.0f;
  g_debug_rpm = 0.0f;

  g_diag_tick++;
}

static uint8_t AS5047P_EvenParity(uint16_t value)
{
  uint8_t parity = 0;

  while (value != 0U)
  {
    parity ^= (uint8_t)(value & 0x0001U);
    value >>= 1U;
  }

  return parity;
}

static uint16_t AS5047P_MakeReadCmd(uint16_t addr)
{
  uint16_t cmd;

  /*
   * AS5047P 读命令格式:
   * bit14 = 1 表示读寄存器, bit15 为偶校验位.
   */
  cmd = 0x4000U | (addr & 0x3FFFU);

  /* 如果当前 16bit 中 1 的个数为奇数, 置 bit15 让整帧变成偶校验. */
  if (AS5047P_EvenParity(cmd))
  {
    cmd |= 0x8000U;
  }

  return cmd;
}

static uint16_t AS5047P_SPI_Transfer16(uint16_t tx_data)
{
#if AS5047_SPI_8BIT_TEST
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];
#endif
  uint16_t rx_data = 0;
  volatile uint32_t delay_count;

#if AS5047_SPI_8BIT_TEST
  tx_buf[0] = (uint8_t)(tx_data >> 8);
  tx_buf[1] = (uint8_t)(tx_data & 0x00FFU);
  rx_buf[0] = 0U;
  rx_buf[1] = 0U;
#endif

  HAL_GPIO_WritePin(AS5047_CS_GPIO_Port,
                    AS5047_CS_Pin,
                    GPIO_PIN_RESET);

  for (delay_count = 0U; delay_count < 200U; delay_count++)
  {
    __NOP();
  }

#if AS5047_SPI_8BIT_TEST
  if (HAL_SPI_TransmitReceive(&hspi2,
                              tx_buf,
                              rx_buf,
                              2,
                              10) != HAL_OK)
#else
  if (HAL_SPI_TransmitReceive(&hspi2,
                              (uint8_t *)&tx_data,
                              (uint8_t *)&rx_data,
                              1,
                              10) != HAL_OK)
#endif
  {
    g_as5047_error_count++;
  }

#if AS5047_SPI_8BIT_TEST
  rx_data = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];
#endif

  for (delay_count = 0U; delay_count < 200U; delay_count++)
  {
    __NOP();
  }

  HAL_GPIO_WritePin(AS5047_CS_GPIO_Port,
                    AS5047_CS_Pin,
                    GPIO_PIN_SET);

  for (delay_count = 0U; delay_count < 200U; delay_count++)
  {
    __NOP();
  }

  g_as5047_last_rx = rx_data;

  if (rx_data == 0U)
  {
    g_as5047_zero_rx_count++;
  }

  return rx_data;
}

static uint16_t AS5047P_ReadReg(uint16_t addr)
{
  uint16_t cmd;
  uint16_t rx;
  uint16_t data;

  cmd = AS5047P_MakeReadCmd(addr);

  /*
   *
   *
   *
   */
  AS5047P_SPI_Transfer16(cmd);
  rx = AS5047P_SPI_Transfer16(0x0000U);
  data = rx & AS5047P_DATA_MASK;

  switch (addr & AS5047P_DATA_MASK)
  {
    case AS5047P_REG_ANGLEUNC:
      g_as5047_angle_raw_frame = rx;

      if (data == 0U)
      {
        g_as5047_zero_angle_count++;
      }
      break;

    case AS5047P_REG_DIAAGC:
      g_as5047_diaagc_raw_frame = rx;

      if (data == 0U)
      {
        g_as5047_zero_diaagc_count++;
      }
      break;

    case AS5047P_REG_MAG:
      g_as5047_mag_raw_frame = rx;

      if (data == 0U)
      {
        g_as5047_zero_mag_count++;
      }
      break;

    default:
      break;
  }

  if ((rx & AS5047P_EF_MASK) != 0U)
  {
    g_as5047_error_count++;

    /*
     *
     */
    AS5047P_SPI_Transfer16(AS5047P_MakeReadCmd(AS5047P_REG_ERRFL));
    AS5047P_SPI_Transfer16(0x0000U);
  }

  return data;
}

static uint16_t AS5047P_ReadRawAngle(void)
{
  uint16_t raw;

  raw = AS5047P_ReadReg(AS5047P_REG_ANGLEUNC);

  g_as5047_raw = raw;
  g_as5047_angle_rad =
      ((float)raw) * SFOC_TWO_PI / 16384.0f;

  g_as5047_angle_deg =
      ((float)raw) * 360.0f / 16384.0f;

  return raw;
}

static void AS5047P_ReadDiagnostics(void)
{
  uint16_t diaagc;
  uint16_t mag;

  diaagc = AS5047P_ReadReg(AS5047P_REG_DIAAGC);
  mag = AS5047P_ReadReg(AS5047P_REG_MAG);

  g_as5047_diaagc_raw = diaagc;
  g_as5047_magl = (uint8_t)((diaagc >> 11) & 0x0001U);
  g_as5047_magh = (uint8_t)((diaagc >> 10) & 0x0001U);
  g_as5047_cof = (uint8_t)((diaagc >> 9) & 0x0001U);
  g_as5047_lf = (uint8_t)((diaagc >> 8) & 0x0001U);
  g_as5047_agc = (uint8_t)(diaagc & 0x00FFU);
  g_as5047_mag_value = mag & AS5047P_DATA_MASK;
}

static void AS5047P_ReadDiagnosticsLowRate(void)
{
  static uint32_t diag_read_div_count = 0U;

  diag_read_div_count++;

  if (diag_read_div_count >= AS5047_DIAG_READ_DIVIDER)
  {
    diag_read_div_count = 0U;
    AS5047P_ReadDiagnostics();
  }
}

static float AS5047P_ReadMechanicalAngleRad(void)
{
  uint16_t raw;

  raw = AS5047P_ReadRawAngle();

  return ((float)raw) * SFOC_TWO_PI / 16384.0f;
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  /* Bring the debug UART up before any peripheral that may call Error_Handler().
   * This makes an early ADC/SPI/TIM initialization failure visible on CH340. */
  uart_init(115200);
  printf("\r\n[EARLY_BOOT] clock=OK sysclk=%lu pclk1=%lu pclk2=%lu\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq(),
         (unsigned long)HAL_RCC_GetPCLK1Freq(),
         (unsigned long)HAL_RCC_GetPCLK2Freq());

  MX_GPIO_Init();
  printf("[EARLY_BOOT] gpio=OK\r\n");
  MX_ADC1_Init();
  printf("[EARLY_BOOT] adc1=OK\r\n");
  MX_SPI2_Init();
  printf("[EARLY_BOOT] spi2=OK\r\n");
#if AS5047_SPI_8BIT_TEST
  HAL_SPI_DeInit(&hspi2);
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  /* Keep the 8-bit transfer compatibility mode at the same conservative
   * AS5047P clock configured by MX_SPI2_Init(). */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  MX_TIM1_Init();
  printf("[EARLY_BOOT] tim1=OK\r\n");
  MX_TIM6_Init();
  printf("[EARLY_BOOT] tim6=OK\r\n");

  /* USER CODE BEGIN 2 */

  Diagnostic_SetNeutralPwm();
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,
                    MOTOR_EN_Pin,
                    MOTOR_SD_DISABLE_LEVEL);
  HAL_GPIO_WritePin(AS5047_CS_GPIO_Port,
                    AS5047_CS_Pin,
                    GPIO_PIN_SET);

  HAL_Delay(100);

  BSP_Init();
  printf("[EARLY_BOOT] bsp_foc=OK entering main loop\r\n");

  /* USER CODE END 2 */

  uint32_t last_status_tick = 0U;
  uint32_t last_current_log_tick = 0U;

  while (1)
  {
    BSP_DebugUartPollCommands();
    BSP_KeyPoll();

    if (FOC_Debug_CurrentCsvEnabled())
    {
      uint32_t vofa_period_ms = FOC_Debug_GetVofaPeriodMs();

      if ((HAL_GetTick() - last_current_log_tick) >= vofa_period_ms)
      {
        last_current_log_tick = HAL_GetTick();
        FOC_Debug_PrintCurrentCsv();
      }
    }
    else if ((HAL_GetTick() - last_status_tick) >= 1000U)
    {
      last_status_tick = HAL_GetTick();
      FOC_Debug_PrintStatus();
    }

    /* 主循环保持 1 ms 轮询，用于串口命令和 VOFA 曲线输出。 */
    HAL_Delay(1);
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue =
      RCC_HSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                          FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Motor_OpenLoop_Tim6Callback(void)
{
  g_tim6_debug_count++;
  Diagnostic_UpdateISR();
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  printf("[ERR] Error_Handler\r\n");
  __disable_irq();

  Motor_OpenLoop_StopPwm();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
