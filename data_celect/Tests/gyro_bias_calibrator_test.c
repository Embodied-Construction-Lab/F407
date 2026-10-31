#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "gyro_bias_calibrator.h"

static void stable_window_and_frozen_motion(void)
{
  GyroBiasCalibrator c;
  GyroBiasCalibrator_Init(&c);
  for (uint64_t t=0; t<=10000; t+=10)
  {
    assert(GyroBiasCalibrator_AddSample(&c, .02f, t));
    if (t<10000) assert(!GyroBiasCalibrator_IsReady(&c));
  }
  assert(GyroBiasCalibrator_IsReady(&c));
  assert(fabsf(GyroBiasCalibrator_BiasDegS(&c)-.02f)<.000001f);
  assert(GyroBiasCalibrator_AddSample(&c, .025f, 10100));
  assert(fabsf(GyroBiasCalibrator_Correct(&c,.025f)-.005f)<.000001f);
  assert(GyroBiasCalibrator_AddSample(&c, -89.98f,10200));
  assert(fabsf(GyroBiasCalibrator_Correct(&c,-89.98f)+90.f)<.00001f);
  GyroBiasCalibrator_ResetPending(&c);
  assert(GyroBiasCalibrator_IsReady(&c));
  assert(!GyroBiasCalibrator_AddSample(&c, .02f, 100));
  assert(GyroBiasCalibrator_AddSample(&c,.02f,110));
}

static void weighted_window(void)
{
  GyroBiasCalibrator c;
  GyroBiasCalibrator_Init(&c);
  assert(GyroBiasCalibrator_AddSample(&c,0.f,0));
  /* Rates occupy unequal time: sample mean != integrated-rate/time. */
  for (uint64_t t=100; t<=10000; t+=100)
  {
    assert(GyroBiasCalibrator_AddSample(&c,.01f,t-90));
    assert(GyroBiasCalibrator_AddSample(&c,.03f,t));
  }
  assert(GyroBiasCalibrator_IsReady(&c));
  assert(fabsf(c.bias_deg_s-.028f)<.000001f);
}

static void invalid_windows(void)
{
  GyroBiasCalibrator c;
  GyroBiasCalibrator_Init(&c);
  for (uint64_t t=0; t<5000;t+=10) GyroBiasCalibrator_AddSample(&c,.02f,t);
  assert(!GyroBiasCalibrator_AddSample(&c,NAN,5000));
  assert(c.sample_count==0);
  GyroBiasCalibrator_AddSample(&c,.02f,6000);
  GyroBiasCalibrator_AddSample(&c,.02f,7000);
  assert(c.sample_count==1); /* Dropped frames cannot count as stationary time. */
  assert(!GyroBiasCalibrator_AddSample(&c,1.f,7010));
  assert(c.sample_count==0);
  for(uint64_t t=0;t<=10000;t+=10) GyroBiasCalibrator_AddSample(&c,(t%20)? .2f:-.2f,t);
  assert(!GyroBiasCalibrator_IsReady(&c));
  GyroBiasCalibrator_Init(&c);
  for(uint64_t t=0;t<=10000;t+=10) GyroBiasCalibrator_AddSample(&c,.15f,t);
  assert(!GyroBiasCalibrator_IsReady(&c));
}
int main(void)
{
  stable_window_and_frozen_motion(); weighted_window(); invalid_windows();
  puts("gyro bias calibrator tests passed"); return 0;
}
