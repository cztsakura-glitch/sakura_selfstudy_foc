/*
 * Minimal GCC fallback for the CMSIS-DSP functions used by this project.
 *
 * The original Keil project links arm_cortexM4lf_math.lib, which is not usable
 * by arm-none-eabi-gcc. If you add a GCC CMSIS-DSP library later, build with:
 *   make USE_CMSIS_DSP_FALLBACK=0 CMSIS_DSP_LIB=/path/to/libarm_cortexM4lf_math.a
 */

#include "arm_math.h"
#include <math.h>

float32_t arm_sin_f32(float32_t x)
{
  return sinf(x);
}

float32_t arm_cos_f32(float32_t x)
{
  return cosf(x);
}
