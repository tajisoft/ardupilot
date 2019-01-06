#include<stdio.h>

#include "mode.h"
#include "Rover.h"

bool ModeBrake::_enter()
{
    // save current wheel encoder rad
    for (int i = 0; i < WHEELENCODER_MAX_INSTANCES; i++) {
        _desired_angle_rad[i] = rover.wheel_encoder_last_angle_rad[i];
        _last_update_ms[i] = rover.wheel_encoder_last_update_ms[i];
    }

    return true;
}

void ModeBrake::update()
{
    for (int i = 0; i < WHEELENCODER_MAX_INSTANCES; i++) {
        float d_angle = rover.wheel_encoder_last_angle_rad[i] - _desired_angle_rad[i];
        // less than 3deg
        if (d_angle < 3.0) {
            continue;
        }

        float throttle = g2.attitude_control.get_throttle_out_from_pitch(-d_angle, 0, g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, rover.G_Dt) * 100.0f;
        g2.motors.set_steering(0.0f);
        g2.motors.set_throttle(throttle);
    }
}
