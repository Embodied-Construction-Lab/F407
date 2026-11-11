/* Included by the existing production-service host test; no MCU dependency. */
static void start_manual(void);
static void FillMotionTelemetry(MotionTelemetry *telemetry, uint32_t now_ms);

/* Test-only instruction-boundary ISR injection around the actual telemetry body. */
static uint32_t fake_primask;
static unsigned telemetry_hook_count, telemetry_interrupt_at, telemetry_interrupt_pending;
static unsigned telemetry_interrupt_fired;

static void apply_encoder_interrupt(void)
{
  sensor_stamp_ms = 198150U;
  sensor_seq = 1857U;
  for (unsigned axis = 0U; axis < FTEPC485_CHANNEL_COUNT; ++axis)
  {
    encoder_data.length_hundredths_mm[axis] = 20000 + (int32_t)axis;
    encoder_data.speed_hundredths_mm_s[axis] = 2000 + (int32_t)axis;
  }
  small_arm_angle_hundredths_deg = 9220;
  encoder_status = FTEPC485_OK;
  telemetry_interrupt_fired = 1U;
}

uint32_t __get_PRIMASK(void) { return fake_primask; }
void __disable_irq(void) { fake_primask = 1U; }
void __set_PRIMASK(uint32_t value)
{
  fake_primask = value;
  if ((fake_primask == 0U) && (telemetry_interrupt_pending != 0U))
  {
    telemetry_interrupt_pending = 0U;
    apply_encoder_interrupt();
  }
}

static void TelemetryInterruptPoint(void)
{
  telemetry_hook_count++;
  if ((telemetry_interrupt_at != 0U) &&
      (telemetry_hook_count == telemetry_interrupt_at))
  {
    if (fake_primask != 0U)
    {
      telemetry_interrupt_pending = 1U;
    }
    else
    {
      apply_encoder_interrupt();
    }
  }
}

static void test_telemetry_encoder_snapshot(void)
{
  unsigned interruptions = 0U;
  const uint32_t rs485_fault_mask = 1UL;
  for (unsigned masked = 0U; masked <= 1U; ++masked)
  {
    for (unsigned point = 1U; point <= 100U; ++point)
    {
      MotionTelemetry telemetry;
      start_manual();
      sensor_seq = 1856U;
      sensor_stamp_ms = 198051U;
      last_sent_sensor_seq = 1856U;
      for (unsigned axis = 0U; axis < FTEPC485_CHANNEL_COUNT; ++axis)
      {
        encoder_data.length_hundredths_mm[axis] = 10000 + (int32_t)axis;
        encoder_data.speed_hundredths_mm_s[axis] = 1000 + (int32_t)axis;
      }
      small_arm_angle_hundredths_deg = 7110;
      encoder_status = FTEPC485_ERROR_UART;
      fake_primask = masked;
      telemetry_hook_count = 0U;
      telemetry_interrupt_at = point;
      telemetry_interrupt_pending = 0U;
      telemetry_interrupt_fired = 0U;
      FillMotionTelemetry(&telemetry, tick);
      assert(fake_primask == masked); /* Never enable previously masked IRQs. */
      interruptions += telemetry_interrupt_fired;
      const unsigned changed = telemetry.sensor_seq == 1857U;
      assert(changed || telemetry.sensor_seq == 1856U);
      assert(telemetry.sensor_stamp_ms == (changed ? 198150U : 198051U));
      assert(telemetry.sensor_is_new == changed);
      assert(telemetry.rs485_ok == changed);
      assert((telemetry.fault_flags & rs485_fault_mask) ==
             (changed ? 0U : rs485_fault_mask));
      assert(fabsf(telemetry.boom_pos_mm - (changed ? 200.00f : 100.00f)) < .0001f);
      assert(fabsf(telemetry.stick_pos_mm - (changed ? 200.01f : 100.01f)) < .0001f);
      assert(fabsf(telemetry.bucket_pos_mm - (changed ? 200.02f : 100.02f)) < .0001f);
      assert(fabsf(telemetry.boom_vel_mmps - (changed ? 20.00f : 10.00f)) < .0001f);
      assert(fabsf(telemetry.stick_vel_mmps - (changed ? 20.01f : 10.01f)) < .0001f);
      assert(fabsf(telemetry.bucket_vel_mmps - (changed ? 20.02f : 10.02f)) < .0001f);
      assert(fabsf(telemetry.arm_angle_deg - (changed ? 92.20f : 71.10f)) < .0001f);
      if (masked != 0U) { assert(telemetry_interrupt_fired == 0U); }
    }
  }
  assert(interruptions > 20U); /* Injection exercised real read boundaries. */
  fake_primask = 0U;
  telemetry_interrupt_at = 0U;
  telemetry_interrupt_pending = 0U;
}
