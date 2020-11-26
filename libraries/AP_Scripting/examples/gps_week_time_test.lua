-- Gps week time test
loop_rate_ms = 1000 / 1
init_yaw = nil
next_yaw = nil
last_sec = 0
action_interval = 15
action_stage = 0
next_action_time = 0
up_speed = 0.2
down_speed = 0.2

function clock_action()
   if init_yaw == nil then
      init_yaw = math.deg(ahrs:get_yaw())
      if init_yaw < 0 then
         init_yaw = init_yaw + 360
      end
      next_yaw = init_yaw
      gcs:send_text(0, string.format("init yaw %f", init_yaw))
   end

   local gps_week_sec = gps:time_week_ms(0) / 1000

   if last_sec == 0 then
      last_sec = gps_week_sec
      return update, loop_rate_ms
   end

   if last_sec ~= gps_week_sec then
      next_yaw = next_yaw + 6
      if next_yaw > 360 then
         next_yaw = next_yaw - 360
      end
      last_sec = gps_week_sec
   end

   local target_vel_ned = Vector3f()
   target_vel_ned:x(0)
   target_vel_ned:y(0)
   target_vel_ned:z(0)
   if not (vehicle:set_target_velocity_NED(target_vel_ned, true, next_yaw)) then
      gcs:send_text(0, "failed to execute velocity command")
   end
end

function climb(spd)
   local target_vel_ned = Vector3f()
   target_vel_ned:x(0)
   target_vel_ned:y(0)
   target_vel_ned:z(spd)
   if not (vehicle:set_target_velocity_NED(target_vel_ned, false, 0)) then
      gcs:send_text(0, "failed to execute velocity command")
   end
end

function update ()
   local sat_count = gps:num_sats(gps:primary_sensor())
   if sat_count < 10 then
      gcs:send_text(1, "gps sats " .. sat_count .. " <10")
      return update, loop_rate_ms
   end

   if next_action_time == 0 then
      next_action_time = uint32_t(1600000000 + param:get("SCR_USER1"))
   end

   gps_week = gps:time_week(gps:primary_sensor())
   gps_week_sec = gps:time_week_ms(gps:primary_sensor()) / 1000
   now = 315932400 + (gps_week * 7 * 24 * 3600) + gps_week_sec
   
   if now < next_action_time then
      gcs:send_text(1, "now " .. tostring(now) .. " wait " .. tostring(next_action_time - now))
   else
      action_stage = action_stage + 1
      next_action_time = next_action_time + action_interval
      gcs:send_text(0, "stage " .. action_stage .. " next " .. tostring(next_action_time))
   end

   -- takeoff stage
   if action_stage == 1 then
      if vehicle:get_mode() ~= 5 then
         vehicle:set_mode(5)
      end
      if not arming:is_armed() then
         arming:arm()
      else
         vehicle:set_mode(4)
      end
      if vehicle:get_mode() == 4 then
         vehicle:start_takeoff(5)
      end
   elseif action_stage == 2 then
      clock_action()
   elseif action_stage == 3 then
      climb(up_speed)
   elseif action_stage == 4 then
      climb(down_speed)
   elseif action_stage == 5 then
      climb(up_speed)
   elseif action_stage == 6 then
      climb(down_speed)
   elseif action_stage == 7 then
      climb(up_speed)
   elseif action_stage == 8 then
      param:set("CIRCLE_RADIUS", 100)
      vehicle:set_mode(7) -- CIRCLE
   elseif action_stage == 9 then
      vehicle:set_mode(9) -- LAND
   end
   
   if action_stage < 10 then
      return update, loop_rate_ms
   end
end

return update, loop_rate_ms 

