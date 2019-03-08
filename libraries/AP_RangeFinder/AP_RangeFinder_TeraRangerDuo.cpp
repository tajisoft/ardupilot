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

    if (!_mode_inited && !set_sensor_mode()) {
        return;
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) ||
        (params.trduo_mode == TeraRangerDuoMode::Fast_Mode && (AP_HAL::millis() - _last_distance_received_ms > RANGEFINDER_TRDUO_FAST_TIMEOUT_MS)) ||
        (params.trduo_mode == TeraRangerDuoMode::Precise_Mode && (AP_HAL::millis() - _last_distance_received_ms > RANGEFINDER_TRDUO_PRECISE_TIMEOUT_MS))) {
        set_status(RangeFinder::RangeFinder_NoData);
    } else {
        set_status(RangeFinder::RangeFinder_Good);
    }
}

bool AP_RangeFinder_TeraRangerDuo::set_sensor_mode()
{
    if (uart == nullptr) {
        return false;
    }

    if (params.trduo_mode == TeraRangerDuoMode::Fast_Mode) {
        uart->write('F');
    } else if (params.trduo_mode == TeraRangerDuoMode::Precise_Mode) {
        uart->write('P');
    }
    // set pinout mode to binary
    uart->write('B');
    
    _mode_inited = true;

    return true;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_RangeFinder_TeraRangerDuo::read_sensor_data()
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t message_count = 0;
    int16_t nbytes = uart->available();

    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == 'T' ) {
            _buffer_count = 0;
        }

        _buffer[_buffer_count++] = c;

        // we should always read 7 bytes TxxSxxCRC
        if (_buffer_count >= 6){
            _buffer_count = 0;

            // check if message has right CRC
            if (crc_crc8(_buffer, 6) == _buffer[6]){
                uint16_t t_distance = process_distance(_buffer[1], _buffer[2]);
                uint16_t s_distance = process_distance(_buffer[4], _buffer[5]);

                hal.console->printf("duo tof distance %d\n", t_distance);
                hal.console->printf("duo sonar distance %d\r\n", s_distance);

                state.distance_cm = (t_distance + s_distance) / 20;
                _last_distance_received_ms = AP_HAL::millis();

                message_count++;
            }
        }
    }
    return (message_count > 0);
}

uint16_t AP_RangeFinder_TeraRangerDuo::process_distance(uint8_t buf1, uint8_t buf2)
{
    return (buf1 << 8) + buf2;
}
