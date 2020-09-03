-- switches between AHRS/EKF sources if the rangefinder distance is less than 10m and the pilot's source selection switch is in the middle
-- this script is intended to help vehicles from between GPS and Non-GPS environments
--
-- setup RCx_OPTION = 89 (EKF Pos Source), move to middle position to enable automatic source selection
-- setup EK3_SRC_ parameters so that GPS is the primary source, Non-GPS (i.e. T265) is the secondary source
-- configure a forward or downward facing lidar with a range of more than 5m
-- SCR_USER1 holds the threshold for rangefinder altitude:
--     if rangefinder distance >= SCR_USER1, source1 will be used
--     if rangefinder distance < SCR_USER1, source2 will be used
-- SCR_USER2 holds the threshold for GPS satellites:
-- SCR_USER3 holds the threshold for GPS hdop:
--     if both GPS satellites >= SCR_USER2 and HDOP <= SCR_USER3, source1 will be used
--     otherwise source2 will be used
--
-- automatic source selection will occur when the auxiliary switch is in middle position
--    GPS will be used when RangeFinder returns distance of >=8m for at least 3 seconds
--    NonGPS will be used when RangeFinder returns distance of <8m for at least 3 seconds

local rangefinder_rotation1 = 0     -- check using forward (0) or downward (25) facing lidar
local rangefinder_rotation2 = 25    -- check using forward (0) or downward (25) facing lidar
local source_counter = 0            -- number of iterations (of 0.1 sec) we have been above or below threshold
local source_counter_max = 20       -- when counter reaches this number (i.e. 2sec) source may be switched
local source_prev = 0               -- previous source, defaults to primary source

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()

  -- check rangefinder distance
  local rangefinder_thresh_dist = param:get('SCR_USER1')  -- SCR_USER1 holds rangefinder threshold
  if rangefinder_thresh_dist and (rangefinder_thresh_dist > 0) then
    -- decide which rangefinder to use
    local rotation_used = rangefinder_rotation1;
    if not rangefinder:has_orientation(rangefinder_rotation1) then
      rotation_used = rangefinder_rotation2
    end

    -- check rangefinder distance
    if rangefinder:has_data_orient(rotation_used) then
      local distance_m = rangefinder:distance_cm_orient(rotation_used) * 0.01
      if (distance_m < rangefinder_thresh_dist) then
        -- rangefinder is below threshold, increase source_counter
        source_counter = math.max(source_counter, 0)
        source_counter = source_counter + 1
        source_counter = math.min(source_counter, source_counter_max)
        --gcs:send_text(0, "RngFnd low (" .. tostring(distance_m) .. "<" .. tostring(rangefinder_thresh_dist))
      else
        -- rangefinder is above threshold, reduce source_counter
        source_counter = math.min(source_counter, 0)
        source_counter = source_counter - 1
        source_counter = math.max(source_counter, -source_counter_max)
        --gcs:send_text(0, "RngFnd high (" .. tostring(distance_m) .. ">" .. tostring(rangefinder_thresh_dist))
      end
    end
  end

  -- check GPS status
  local gps_sat_thresh = param:get('SCR_USER2')     -- SCR_USER2 holds GPS satellite count threshold
  local gps_hdop_thresh = param:get('SCR_USER3')    -- SCR_USER3 holds GPS hdop threshold
  local gps_speedaccuracy_thresh = param:get('SCR_USER4')  -- SCR_USER4 holds GPS speed accuracy threshold
  if gps_sat_thresh and gps_hdop_thresh and gps_speedaccuracy_thresh then
    -- satellite count vote (vote "-1" to move towards GPS, "+1" to move to Non-GPS)
    local gps_sat_vote = 0
    if (gps_sat_thresh > 0) then
      if (gps:num_sats(0) < gps_sat_thresh) then
        gps_sat_vote = 1
        --gcs:send_text(0, "GPS Sat low (" .. tostring(gps:num_sats(0)) .. "<" .. tostring(gps_sat_thresh) .. ")")
      else
        gps_sat_vote = -1
        --gcs:send_text(0, "GPS Sat high (" .. tostring(gps:num_sats(0)) .. ">" .. tostring(gps_sat_thresh) .. ")")
      end
    end
    -- hdop check vote (vote "-1" to move towards GPS, "+1" to move to Non-GPS)
    local gps_hdop_vote = 0
    if (gps_hdop_thresh > 0) then
      if (gps:get_hdop(0) > gps_hdop_thresh) then
        gps_hdop_vote = 1
        --gcs:send_text(0, "GPS Hdop high (" .. tostring(gps:get_hdop(0)) .. ">" .. tostring(gps_hdop_thresh) .. ")")
      else
        gps_hdop_vote = -1
        --gcs:send_text(0, "GPS Hdop low (" .. tostring(gps:get_hdop(0)) .. "<" .. tostring(gps_hdop_thresh) .. ")")
      end
    end
	-- speed accuracy check vote (vote "-1" to move towards GPS, "+1" to move to Non-GPS)
    local gps_speedaccuracy_vote = 0
    if (gps_speedaccuracy_thresh > 0 and gps:speed_accuracy(0)) then
      if (gps:speed_accuracy(0) > gps_speedaccuracy_thresh) then
        gps_speedaccuracy_vote = 1
        --gcs:send_text(0, "GPS SpdAcc high (" .. tostring(gps:speed_accuracy(0)) .. ">" .. tostring(gps_speedaccuracy_thresh) .. ")")
      else
        gps_speedaccuracy_vote = -1
        --gcs:send_text(0, "GPS SpdAcc low (" .. tostring(gps:speed_accuracy(0)) .. "<" .. tostring(gps_speedaccuracy_thresh) .. ")")
      end
    end
    -- combine votes (a vote of "0" means no vote)
    if gps_sat_vote == 0 then
      if gps_hdop_vote ~= 0 then
        gps_sat_vote = gps_hdop_vote
      else
        gps_sat_vote = gps_speedaccuracy_vote
      end
    end
    if gps_hdop_vote == 0 then
      if gps_sat_vote ~= 0 then
        gps_hdop_vote = gps_sat_vote
      else
        gps_hdop_vote = gps_speedaccuracy_vote
      end
    end
    if gps_speedaccuracy_vote == 0 then
      if gps_sat_vote ~= 0 then
        gps_speedaccuracy_vote = gps_sat_vote
      else
        gps_speedaccuracy_vote = gps_hdop_vote
      end
    end
    if (gps_sat_vote > 0 or gps_hdop_vote > 0 or gps_speedaccuracy_vote > 0) then
      -- satellite count, hdop or speed accuracy is not good so move towards Non-GPS
      source_counter = math.max(source_counter, 0)
      source_counter = source_counter + 1
      source_counter = math.min(source_counter, source_counter_max)
    elseif (gps_sat_vote < 0 and gps_hdop_vote < 0 and gps_speedaccuracy_vote < 0) then
      -- satellite count, hdop and speed accuracy are good (or unchecked) so move towards GPS
      source_counter = math.min(source_counter, 0)
      source_counter = source_counter - 1
      source_counter = math.max(source_counter, -source_counter_max)
    end
  end

  -- read switch input from RCx_FUNCTION = 89 (EKF Source Select)
  local rc_function_source = rc:find_channel_for_option(89)
  if rc_function_source then
    local sw_pos = rc_function_source:get_aux_switch_pos()
    if (sw_pos == 0) then
      -- pilot has manual selected primary source
      if (source_prev ~= 0) then
        source_prev = 0
        gcs:send_text(0, "Pilot switched to Primary source")
      end
    elseif (sw_pos == 1) then
      -- pilot has selected auto section of source
      if (source_prev == 1) and (source_counter == -source_counter_max) then
        -- switch to primary source
        ahrs:set_position_source(0)
        source_prev = 0
        gcs:send_text(0, "AHRS switched to Primary source")
      end
      if (source_prev == 0) and (source_counter == source_counter_max) then
        -- switch to secondary source
        ahrs:set_position_source(1)
        source_prev = 1
        gcs:send_text(0, "AHRS switched to Secondary source")
      end
      --gcs:send_text(0, "Src:" .. tostring(source_prev) .. "Cnt:" .. tostring(source_counter))
    else
      -- pilot has manual selected secondary source
      if (source_prev ~= 1) then
        source_prev = 1
        gcs:send_text(0, "Pilot switched to Secondary source")
      end
    end
  end

  return update, 100
end

return update()
