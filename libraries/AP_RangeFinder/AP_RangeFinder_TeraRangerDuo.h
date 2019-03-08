#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define RANGEFINDER_TRDUO_FAST_TIMEOUT_MS            200  // requests timeout after 0.3 seconds for fast mode
#define RANGEFINDER_TRDUO_PRECISE_TIMEOUT_MS         1200  // requests timeout after 0.3 seconds for precise mode

class AP_RangeFinder_TeraRangerDuo : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_TeraRangerDuo(RangeFinder::RangeFinder_State &_state,
                                 AP_RangeFinder_Params &_params,
                                 AP_SerialManager &serial_manager,
                                 uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void) override;

    // sensor mode
    enum TeraRangerDuoMode {
        Precise_Mode = 0,
        Fast_Mode    = 1
    };

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // set sensor mode
    bool set_sensor_mode();
    // check and process replies from sensor
    bool read_sensor_data();
    uint16_t process_distance(uint8_t buf1, uint8_t buf2);

    // reply related variables
    AP_HAL::UARTDriver *uart = nullptr;
    
    // mode init flag
    bool _mode_inited;
    uint8_t _buffer_count;
    uint8_t _buffer[7];
    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
};
