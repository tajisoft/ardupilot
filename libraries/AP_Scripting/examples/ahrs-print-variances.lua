-- This script displays the ahrs variances at 1hz

function update() -- this is the loop which periodically runs)
  vel_variance, pos_variance, height_variance, mag_variance, airspeed_variance, offset = ahrs:get_variances()
  if vel_variance then
    gcs:send_text(0, string.format("Variances Pos:%.1f Vel:%.1f Hgt:%.1f Mag:%.1f", pos_variance, vel_variance, height_variance, mag_variance:length()))
  else
    gcs:send_text(0, string.format("Failed to retrieve variances"))
  end
  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
