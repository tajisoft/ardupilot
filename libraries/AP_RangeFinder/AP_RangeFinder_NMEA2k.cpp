#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_RangeFinder_NMEA2K.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/sonner/WaterDepth.hpp>

extern const AP_HAL::HAL& hal;

#define debug_water_depth_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(WaterDepthCb, uavcan::equipment::sonner::WaterDepth);

AP_RangeFinder_NMEA2K::DetectedModules AP_RangeFinder_NMEA2K::_detected_modules[] = {0};
HAL_Semaphore AP_RangeFinder_NMEA2K::_sem_registry;

/*
  constructor - registers instance at top Baro driver
 */
AP_RangeFinder_NMEA2K::AP_RangeFinder_NMEA2K(RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_state)
{}

void AP_RangeFinder_NMEA2K::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::sonner::WaterDepth, WaterDepthCb> *water_depth_listener;
    water_depth_listener = new uavcan::Subscriber<uavcan::equipment::sonner::WaterDepth, WaterDepthCb>(*node);
    // Msg Handler
    const int water_depth_listener_res = water_depth_listener->start(WaterDepthCb(ap_uavcan, &handle_water_depth));
    if (water_depth_listener_res < 0) {
        AP_HAL::panic("UAVCAN WaterDepth subscriber start problem\n\r");
        return;
    }
}

bool AP_RangeFinder_NMEA2K::take_registry()
{
    return _sem_registry.take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_RangeFinder_NMEA2K::give_registry()
{
    _sem_registry.give();
}

// AP_RangeFinder_Backend* AP_RangeFinder_NMEA2K::probe(RangeFinder &rangefinder)
// {
//     if (!take_registry()) {
//         return nullptr;
//     }
//     AP_RangeFinder_Backend* backend = nullptr;
//     for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
//         if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
//             backend = new AP_RangeFinder_NMEA2K(rangefinder);
//             if (backend == nullptr) {
//                 debug_water_depth_uavcan(2,
//                                   _detected_modules[i].ap_uavcan->get_driver_index(),
//                                   "Failed register UAVCAN Baro Node %d on Bus %d\n",
//                                   _detected_modules[i].node_id,
//                                   _detected_modules[i].ap_uavcan->get_driver_index());
//             } else {
//                 _detected_modules[i].driver = backend;
//                 backend->_ap_uavcan = _detected_modules[i].ap_uavcan;
//                 backend->_node_id = _detected_modules[i].node_id;
//                 backend->register_sensor();
//                 debug_water_depth_uavcan(2,
//                                   _detected_modules[i].ap_uavcan->get_driver_index(),
//                                   "Registered UAVCAN Baro Node %d on Bus %d\n",
//                                   _detected_modules[i].node_id,
//                                   _detected_modules[i].ap_uavcan->get_driver_index());
//             }
//             break;
//         }
//     }
//     give_registry();
//     return backend;
// }

AP_RangeFinder_NMEA2K* AP_RangeFinder_NMEA2K::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        //Check if there's an empty spot for possible registeration
        for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
            if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
                //Already Detected
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
                if (_detected_modules[i].ap_uavcan == nullptr) {
                    _detected_modules[i].ap_uavcan = ap_uavcan;
                    _detected_modules[i].node_id = node_id;
                    break;
                }
            }
        }
    }

    return nullptr;
}

void AP_RangeFinder_NMEA2K::handle_water_depth(AP_UAVCAN* ap_uavcan, uint8_t node_id, const WaterDepthCb &cb)
{
    if (take_registry()) {
        AP_RangeFinder_NMEA2K* driver = get_uavcan_backend(ap_uavcan, node_id, true);
        if (driver == nullptr) {
            give_registry();
            return;
        }
        {
            WITH_SEMAPHORE(driver->_sem_baro);
            driver->_water_depth = cb.msg->water_depth;
            driver->new_water_depth = true;
            driver->_last_timestamp = AP_HAL::millis64();
        }
        give_registry();
    }
}

// Read the sensor
void AP_RangeFinder_NMEA2K::update(void)
{
    WITH_SEMAPHORE(_sem_baro);
    if (new_water_depth) {
        state.last_reading_ms = _last_timestamp;
        state.distance_cm = _water_depth;
        new_water_depth = false;
    }
}

#endif // HAL_WITH_UAVCAN
