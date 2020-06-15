#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller

#define AC_SUCKER_ACCEL_CMSS_MAX         100.0f  // maximum acceleration/deceleration in cm/s/s used to adjust angle

// bit masks for enabled fence types.
#define AC_SUCKER_DISABLED               0       // avoidance disabled
#define AC_SUCKER_DEFAULT                0       // default disabled

// definitions for sucker adjust
#define AC_SUCKER_IGNORE_MIN_ANGLE    5.0f    // ignore less than 5 degree

#define AC_SUCKER_ACTIVE_LIMIT_TIMEOUT_MS    500     // if limiting is active if last limit is happend in the last x ms

/*
 * This class prevents the vehicle from leaving a polygon fence in
 * 2 dimensions by limiting velocity (adjust_velocity).
 */
class AC_Sucker {
public:
    AC_Sucker();

    /* Do not allow copies */
    AC_Sucker(const AC_Sucker &other) = delete;
    AC_Sucker &operator=(const AC_Sucker&) = delete;

    // get singleton instance
    static AC_Sucker *get_singleton() {
        return _singleton;
    }

    // return true if any avoidance feature is enabled
    bool enabled() const { return _enabled != AC_SUCKER_DISABLED; }

    void run();

    // /*
    //  * Adjusts the desired velocity so that the vehicle can stop
    //  * before the fence/object.
    //  * Note: Vector3f version is for convenience and only adjusts x and y axis
    //  */
    // void adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
    // void adjust_velocity(float kP, float accel_cmss, Vector3f &desired_vel_cms, float dt);

    // // adjust desired horizontal speed so that the vehicle stops before the fence or object
    // // accel (maximum acceleration/deceleration) is in m/s/s
    // // heading is in radians
    // // speed is in m/s
    // // kP should be zero for linear response, non-zero for non-linear response
    // // dt is the time since the last call in seconds
    // void adjust_speed(float kP, float accel, float heading, float &speed, float dt);

    // // adjust vertical climb rate so vehicle does not break the vertical fence
    // void adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float dt);

    // // adjust roll-pitch to push vehicle away from objects
    // // roll and pitch value are in centi-degrees
    // // angle_max is the user defined maximum lean angle for the vehicle in centi-degrees
    // void adjust_roll_pitch(float &roll, float &pitch, float angle_max);

    // // enable/disable proximity based avoidance
    // void proximity_avoidance_enable(bool on_off) { _proximity_enabled = on_off; }
    // bool proximity_avoidance_enabled() { return _proximity_enabled; }

    // // helper functions

    // // Limits the component of desired_vel_cms in the direction of the unit vector
    // // limit_direction to be at most the maximum speed permitted by the limit_distance_cm.
    // // uses velocity adjustment idea from Randy's second email on this thread:
    // //   https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
    // void limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt);

    //  // compute the speed such that the stopping distance of the vehicle will
    //  // be exactly the input distance.
    //  // kP should be non-zero for Copter which has a non-linear response
    // float get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const;

    // // return margin (in meters) that the vehicle should stay from objects
    // float get_margin() const { return _margin; }

    // // return true if limiting is active
    // bool limits_active() const {return (AP_HAL::millis() - _last_limit_time) < AC_SUCKER_ACTIVE_LIMIT_TIMEOUT_MS;};

    static const struct AP_Param::GroupInfo var_info[];

private:

    // /*
    //  * Adjusts the desired velocity for the circular fence.
    //  */
    // void adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    // /*
    //  * Adjusts the desired velocity for inclusion and exclusion polygon fences
    //  */
    // void adjust_velocity_inclusion_and_exclusion_polygons(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    // /*
    //  * Adjusts the desired velocity for the inclusion and exclusion circles
    //  */
    // void adjust_velocity_inclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
    // void adjust_velocity_exclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    // /*
    //  * Adjusts the desired velocity for the beacon fence.
    //  */
    // void adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    // /*
    //  * Adjusts the desired velocity based on output from the proximity sensor
    //  */
    // void adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    // /*
    //  * Adjusts the desired velocity given an array of boundary points
    //  *   earth_frame should be true if boundary is in earth-frame, false for body-frame
    //  *   margin is the distance (in meters) that the vehicle should stop short of the polygon
    //  *   stay_inside should be true for fences, false for exclusion polygons
    //  */
    // void adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f* boundary, uint16_t num_points, bool earth_frame, float margin, float dt, bool stay_inside);

    // /*
    //  * Computes distance required to stop, given current speed.
    //  */
    // float get_stopping_distance(float kP, float accel_cmss, float speed_cms) const;

    // /*
    //  * methods for avoidance in non-GPS flight modes
    //  */

    // // convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
    // float distance_to_lean_pct(float dist_m);

    // // returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
    // void get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative);

    // parameters
    AP_Int8 _enabled;
    AP_Int16 _ignore_angle_min; // minimum angle to ignore
    AP_Float _dist_left;
    AP_Float _dist_right;
    // AP_Float _dist_max;         // distance (in meters) from object at which obstacle avoidance will begin in non-GPS modes
    // AP_Float _margin;           // vehicle will attempt to stay this distance (in meters) from objects while in GPS modes
    // AP_Int8 _behavior;          // avoidance behaviour (slide or stop)

    // bool _proximity_enabled = true; // true if proximity sensor based avoidance is enabled (used to allow pilot to enable/disable)
    uint32_t _last_limit_time;      // the last time a limit was active

    static AC_Sucker *_singleton;
};

namespace AP {
    AC_Sucker *ac_sucker();
};
