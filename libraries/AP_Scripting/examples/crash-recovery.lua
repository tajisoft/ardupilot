-- This script try to detect crash on air and change flight mode to help recovery.
-- Crash detection conditions.
-- ATT value, Accel value, EKF variance

local CRASH_LEVEL_LOW = 0
local CRASH_LEVEL_WARN = 1
local CRASH_LEVEL_HIGH = 2
local RECOVERY_MODE_STAGE1 = 2          -- ALT_HOLD mode for crash detected
local RECOVERY_MODE_STAGE2 = 0          -- STABILIZE mode for recovery timeout or CRASH_LEVEL_HIGH
local ENABLE_CRASH_RECOVERY_CH = 301    -- RCx_OPTION: 301 (Scription2) to enable crash recovery
local GMASS = 9.80665                   -- Gravity acceleration in m/s/s
local ACC_LOW_THRESHOLD = 1.5 * GMASS
local ACC_WARN_THRESHOLD = 2.0 * GMASS

local enable = false              -- Enable crash recovery
local prev_mode = -1              -- Mode to restore after vehicle stabilized
local timeout = 10 * 1000         -- We have 10s for recovery. If we fail to recovery within timer change mode to STAGE2.
local timer_start = -1            -- Timer started
local prev_sw_pos = -1            -- Previous switch position

function init()
  local mode_stage1 = param:get('SCR_USER4')
  if mode_stage1 ~= nil and mode_stage1 > 0 then
    RECOVERY_MODE_STAGE1 = mode_stage1
  end
  gcs:send_text(6, string.format("Crash Recovery: init stage1 mode %d, stage2 mode %d", RECOVERY_MODE_STAGE1, RECOVERY_MODE_STAGE2))

  if not ahrs:initialised() or not ahrs:home_is_set() then
    gcs:send_text(6, "Crash Recovery: vehicle not ready.")
    return init, 1000
  else
    return update, 1000
  end
end

function play_notify_tune()
  if enable then
    notify:play_tune("L8C")       -- one long lower tone
  else
    notify:play_tune("L16FFF")    -- three very fast, high tones
  end
end

function get_crash_level()
  -- only check acceleration
  local cur_acc = ahrs:get_accel()
  local acc = cur_acc:length()
  -- gcs:send_text(6, string.format("acc %.2f", acc))
  if acc <= ACC_LOW_THRESHOLD then
    return CRASH_LEVEL_LOW
  elseif acc > ACC_LOW_THRESHOLD and acc <= ACC_WARN_THRESHOLD then
    return CRASH_LEVEL_WARN
  else
    return CRASH_LEVEL_HIGH
  end
end

function update() -- 100Hz loop
  -- check switch is configured
  local rc_function = rc:find_channel_for_option(ENABLE_CRASH_RECOVERY_CH)
  if rc_function == nil then
    gcs:send_text(0, "Crash recovery: RXx_OPTION=301 not set.")
  else
    local sw_enable_pos = rc_function:get_aux_switch_pos()
    if prev_sw_pos ~= sw_enable_pos then
      prev_sw_pos = sw_enable_pos
      if sw_enable_pos == 2 then
        enable = true
        gcs:send_text(6, "Crash Recovery: enabled")
      else
        enable = false
        gcs:send_text(6, "Crash Recovery: disabled")
      end
      play_notify_tune()
    else
    end
  end

  if not enable then
    return update, 1000
  end

  crash_level = get_crash_level()
  if crash_level == CRASH_LEVEL_LOW then
    if prev_mode ~= -1 and vehicle:get_mode() ~= prev_mode then
      vehicle:set_mode(prev_mode)
      gcs:send_text(6, string.format("Crash level: LOW change mode to %d", prev_mode))
      prev_mode = -1
    end
    timer_start = -1
  else
    -- store current mode
    if prev_mode == -1 then
      prev_mode = vehicle:get_mode()
    end
    -- check recovery timeout
    if timer_start == -1 then
      timer_start = millis()
    else
      if millis() - timer_start >= timeout and vehicle:get_mode() ~= RECOVERY_MODE_STAGE2 then
        vehicle:set_mode(RECOVERY_MODE_STAGE2)
        gcs:send_text(0, string.format("Recovery timeout! Change mode to %d", RECOVERY_MODE_STAGE2))
        return update, 10
      end
    end
    if crash_level == CRASH_LEVEL_WARN and vehicle:get_mode() ~= RECOVERY_MODE_STAGE1 then
      vehicle:set_mode(RECOVERY_MODE_STAGE1)
      gcs:send_text(5, string.format("Crash level: WARN change mode to %d", RECOVERY_MODE_STAGE1))
    end
    if crash_level == CRASH_LEVEL_HIGH and vehicle:get_mode() ~= RECOVERY_MODE_STAGE2 then
      vehicle:set_mode(RECOVERY_MODE_STAGE2)
      gcs:send_text(0, string.format("Crash level: HIGH change mode to %d", RECOVERY_MODE_STAGE2))
    end
  end
  return update, 10 -- reschedules the loop
end

return init() -- initialize first
