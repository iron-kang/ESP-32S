#include "utility.h"

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

uint16_t single2half(float number)
{
    uint32_t num = *((uint32_t*)&number);
    uint32_t s = num>>31;
    uint32_t e = (num>>23)&0x0FF;

    if ((e==255) && (num&0x007fffff))
        return 0x7E00; // NaN
    if (e>(127+15))
        return s?0xFC00:0x7C00;  //+/- inf
    if (e<(127-15))
        return 0; //Do not handle generating subnormalised representation

    return (s<<15) | ((e-127+15)<<10) | (((num>>13)&0x3FF)+((num>>12)&0x01));
}

float half2single(uint16_t number)
{
    uint32_t fp32;
    uint32_t s = number>>15;
    uint32_t e = (number>>10)&0x01F;

    //All binary16 can be mapped in a binary32
    if(e==0)
        e=15-127;
    if (e==0x1F)
    {
        if (number&0x03FF)
            fp32 = 0x7FC00000; // NaN
        else
            fp32 = s?0xFF800000:0x7F800000;  //+/- inf
    }
    else
        fp32 = (s<<31) | ((e+127-15)<<23) | ((number&0x3ff)<<13);

    return *(float*)&fp32;
}


/*****************************************************************************/

uint16_t limitUint16(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

