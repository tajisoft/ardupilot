#include<stdio.h>

#include "mode.h"
#include "Rover.h"

bool ModeBrake::_enter()
{
    if (rover.is_boat() || !rover.have_position) {
        return false;
    }

    _brake_heading_cd = ahrs.yaw_sensor;
    _brake_point = rover.current_loc;
    _brake_pitch = ahrs.pitch_sensor;

    return true;
}

void ModeBrake::update()
{
    if (get_distance_cm(rover.current_loc, _brake_point) > 5) {
        set_desired_location(_brake_point);
    }
    int32_t diff_pitch = ahrs.pitch_sensor - _brake_pitch;
    if (abs(diff_pitch) > 5) {
        calc_throttle((diff_pitch < 0 ? -diff_pitch : diff_pitch) * 10, false, true);
    }
    calc_steering_to_heading(_brake_heading_cd);
}
