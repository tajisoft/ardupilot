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
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_RangeFinder_TeraRangerDuo::AP_RangeFinder_TeraRangerDuo(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_SerialManager &serial_manager,
                                                           uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params),
    _mode_inited(false)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

// detect if a TeraRanger Tower proximity sensor is connected by looking for a configured serial port
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

    if (!_mode_inited) {
        set_sensor_mode();
        return;
    }

    // process incoming messages
    if (read_sensor_data()) {
        if ((int16_t)state.distance_cm > params.max_distance_cm) {
            gcs().send_text(MAV_SEVERITY_INFO, "TERA outrange high");
            set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
        } else if ((int16_t)state.distance_cm < params.min_distance_cm) {
            set_status(RangeFinder::RangeFinder_OutOfRangeLow);
            gcs().send_text(MAV_SEVERITY_INFO, "TERA outrange low");
        } else {
            set_status(RangeFinder::RangeFinder_Good);
            gcs().send_text(MAV_SEVERITY_INFO, "TERA outrange good");
        }
    } else if (AP_HAL::millis() - state.last_reading_ms > RANGEFINDER_TRDUO_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);
        gcs().send_text(MAV_SEVERITY_INFO, "TERA outrange no data");
    }
}

void AP_RangeFinder_TeraRangerDuo::set_sensor_mode()
{
    if (uart == nullptr) {
        return;
    }

    // set preceise mode
    uart->write('P');
    // set pinout mode to binary
    uart->write('B');
    uart->flush();

    _mode_inited = true;
    gcs().send_text(MAV_SEVERITY_INFO, "TERA mode_inited");
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_RangeFinder_TeraRangerDuo::read_sensor_data()
{
    if (uart == nullptr) {
        return false;
    }

    bool found_start = false;
    uint16_t message_count = 0;
    int16_t nbytes = uart->available();

    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == 'T') {
            _buffer_count = 0;
            found_start = true;
            gcs().send_text(MAV_SEVERITY_NOTICE, "TERA found T");
        }

        if (!found_start) {
            continue;
        }

        _buffer[_buffer_count++] = c;
        
        if (_buffer_count >= TERARANGER_DUO_BUFFER_SIZE_FULL) {
            gcs().send_text(MAV_SEVERITY_INFO, "TERA T %x %x %x %x %x %x %x", _buffer[0], _buffer[1], _buffer[2], _buffer[3], _buffer[4], _buffer[5], _buffer[6]);
            // check if message has right CRC
            if (crc_crc8(_buffer, TERARANGER_DUO_BUFFER_SIZE_FULL - 1) == _buffer[TERARANGER_DUO_BUFFER_SIZE_FULL - 1]){
                uint16_t t_distance = process_distance(_buffer[1], _buffer[2]);
                uint16_t s_distance = process_distance(_buffer[4], _buffer[5]);
                
                state.distance_cm = (t_distance + s_distance) / 2;

                gcs().send_text(MAV_SEVERITY_INFO, "TERA %d + %d / 2 = %d", t_distance, s_distance, state.distance_cm);
                state.last_reading_ms = AP_HAL::millis();
            }
            message_count++;
            _buffer_count = 0;
            found_start = false;
        }
    }
    return (message_count > 0);
}

uint16_t AP_RangeFinder_TeraRangerDuo::process_distance(uint8_t buf1, uint8_t buf2)
{
    uint16_t val = buf1 << 8;
    val |= buf2;

    return val / TERARANGER_DUO_VALUE_TO_CM_FACTOR;
}
