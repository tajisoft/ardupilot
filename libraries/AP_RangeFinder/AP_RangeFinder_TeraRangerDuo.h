#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define TERARANGER_DUO_BUFFER_SIZE_FULL 7
#define TERARANGER_DUO_VALUE_TO_CM_FACTOR 10
#define TERARANGER_DUO_MIN_DISTANCE_TOF 20
#define TERARANGER_DUO_MAX_DISTANCE_TOF 1400
#define TERARANGER_DUO_MIN_DISTANCE_ULTRASOUND 5
#define TERARANGER_DUO_MAX_DISTANCE_ULTRASOUND 765
#define RANGEFINDER_TRDUO_TIMEOUT_MS 500  // timeout 0.3 seconds

class AP_RangeFinder_TeraRangerDuo : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_TeraRangerDuo(RangeFinder::RangeFinder_State &_state,
                                 AP_SerialManager &serial_manager,
                                 uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);
    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // check and process replies from sensor
    bool get_reading(uint16_t &distance_cm);
    uint16_t process_distance(uint8_t buf1, uint8_t buf2);
    bool is_valid_range(uint16_t distance, bool is_sound);
    uint16_t average(bool is_sound);
    void update_status();

    // uart driver
    AP_HAL::UARTDriver *uart = nullptr;
    
    uint32_t _last_reading_ms;
    uint32_t _last_reading_tof_ms;
    uint32_t _last_reading_sound_ms;

    // buffer
    uint8_t _buffer[7];
    uint8_t _buffer_count;
    bool _found_start;
    // moving average
    uint16_t _average_tof[5];
    uint16_t _average_sound[5];
    uint8_t _tof_count;
    uint8_t _sound_count;
};
