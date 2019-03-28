/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_TeraRangerDuo.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include <ctype.h>
#include <stdio.h>

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_TeraRangerDuo::AP_RangeFinder_TeraRangerDuo(RangeFinder::RangeFinder_State &_state,
                                                           AP_SerialManager &serial_manager,
                                                           uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state),
    _tof_count(0),
    _sound_count(0)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

// detect if a TeraRanger DUO rangefinder sensor is connected by looking for a configured serial port
bool AP_RangeFinder_TeraRangerDuo::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// update the state of the sensor
void AP_RangeFinder_TeraRangerDuo::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // process incoming messages
    if (get_reading(state.distance_cm)) {
        _last_reading_ms = AP_HAL::millis();        
    }
    update_status();
}

// update status
void AP_RangeFinder_TeraRangerDuo::update_status()
{
    if (AP_HAL::millis() - _last_reading_ms > RANGEFINDER_TRDUO_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);
    } else {
        if ((int16_t)state.distance_cm > TERARANGER_DUO_MAX_DISTANCE_TOF) {
            set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
        } else if ((int16_t)state.distance_cm < TERARANGER_DUO_MIN_DISTANCE_ULTRASOUND) {
            set_status(RangeFinder::RangeFinder_OutOfRangeLow);
        } else {
            set_status(RangeFinder::RangeFinder_Good);
        }
    }
}

// read - return last value measured by sensor
bool AP_RangeFinder_TeraRangerDuo::get_reading(uint16_t &distance_cm)
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t message_count = 0;
    int16_t nbytes = uart->available();

    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == 'T') {
            _buffer_count = 0;
            _found_start = true;
        }

        if (!_found_start) {
            continue;
        }

        _buffer[_buffer_count++] = c;
        
        if (_buffer_count >= TERARANGER_DUO_BUFFER_SIZE_FULL) {
            // check if message has right CRC
            if (crc_crc8(_buffer, TERARANGER_DUO_BUFFER_SIZE_FULL - 1) == _buffer[TERARANGER_DUO_BUFFER_SIZE_FULL - 1]){
                uint16_t t_distance = process_distance(_buffer[1], _buffer[2]);
                uint16_t s_distance = process_distance(_buffer[4], _buffer[5]);

                // usualy we use sound range
                uint32_t now = AP_HAL::millis();
                gcs().send_text(MAV_SEVERITY_INFO, "distance sound %d tof %d", s_distance, t_distance);
                if (is_valid_range(s_distance, true)) {
                    if (now - _last_reading_sound_ms > RANGEFINDER_TRDUO_TIMEOUT_MS) {
                        _sound_count = 0;
                    }
                    _average_sound[_sound_count++] = s_distance;
                    distance_cm = average(true);
                    gcs().send_text(MAV_SEVERITY_INFO, "valid sound count %d average %d", _sound_count, s_distance);
                    _last_reading_ms = now;
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "invalid sound");
                    if (is_valid_range(t_distance, false)) {
                        if (now - _last_reading_tof_ms > RANGEFINDER_TRDUO_TIMEOUT_MS) {
                            _tof_count = 0;
                        }
                        _average_tof[_tof_count++] = t_distance;
                        distance_cm = average(false);
                        gcs().send_text(MAV_SEVERITY_INFO, "valid sound count %d average %d", _tof_count, t_distance);
                        _last_reading_ms = now;
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "invalid tof");
                    }
                }
            }
            message_count++;
            _buffer_count = 0;
            _found_start = false;
        }
    }
    return (message_count > 0);
}

uint16_t AP_RangeFinder_TeraRangerDuo::average(bool is_sound)
{
    uint8_t idx = _tof_count;
    uint32_t total = 0;
    if (is_sound) {
        idx = _sound_count;
    }
    while (idx-- > 0) {
        if (is_sound) {
            total += _average_sound[idx - 1];
        } else {
            total += _average_tof[idx - 1];
        }
    }
    
    if (is_sound) {
        return total / _sound_count;
    } else {
        return total / _tof_count;
    }
}

bool AP_RangeFinder_TeraRangerDuo::is_valid_range(uint16_t distance, bool is_sound)
{
    if (is_sound) {
        return distance >= TERARANGER_DUO_MIN_DISTANCE_ULTRASOUND && distance <= TERARANGER_DUO_MAX_DISTANCE_ULTRASOUND;
    } else {
        return distance >= TERARANGER_DUO_MIN_DISTANCE_TOF && distance <= TERARANGER_DUO_MAX_DISTANCE_TOF;
    }
}

uint16_t AP_RangeFinder_TeraRangerDuo::process_distance(uint8_t buf1, uint8_t buf2)
{
    uint16_t val = buf1 << 8;
    val |= buf2;

    return val / TERARANGER_DUO_VALUE_TO_CM_FACTOR;
}
