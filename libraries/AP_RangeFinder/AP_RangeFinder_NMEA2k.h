#pragma once

#include "RangeFinder_Backend.h"
#include "CAN.h"

#include <AP_Common/Semaphore.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

class WaterDepthCb;

class AP_RangeFinder_NMEA2K : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_NMEA2K(RangeFinder::RangeFinder_State &_state);

    bool detect(CANManager *_manager);

    void update() override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_RangeFinder_NMEA2K* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new);
    static AP_RangeFinder_Backend* probe(RangeFinder &rangefinder);

    static void handle_water_depth(AP_UAVCAN* ap_uavcan, uint8_t node_id, const WaterDepthCb &cb);

private:
    static bool take_registry();
    static void give_registry();

    bool new_water_depth;
    float _water_depth;
    uint64_t _last_timestamp;

    HAL_Semaphore _sem_baro;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_RangeFinder_NMEA2K* driver;
    } _detected_modules[RANGEFINDER_MAX_INSTANCES];

    static HAL_Semaphore _sem_registry;
};
