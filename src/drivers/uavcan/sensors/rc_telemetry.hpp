// Copyright (c) 2026 Noctua contributors. SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "sensor_bridge.hpp"
#include <uavcan/tunnel/Targetted.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
class UavcanRcTelemetry : public IUavcanSensorBridge {
public:
    explicit UavcanRcTelemetry(uavcan::INode &node) : _publisher(node) {}
    const char *get_name() const override { return "rc_telemetry"; }
    int init() override;
    void update() override;
    unsigned get_num_redundant_channels() const override { return 0; }
    void print_status() const override;
private:
    uavcan::Publisher<uavcan::tunnel::Targetted> _publisher;
    uORB::Subscription _rc{ORB_ID(input_rc)};
    uORB::Subscription _battery{ORB_ID(battery_status)};
    uORB::Subscription _gps{ORB_ID(vehicle_gps_position)};
    uORB::Subscription _attitude{ORB_ID(vehicle_attitude)};
    uORB::Subscription _status{ORB_ID(vehicle_status)};
    int32_t _target{0};
    uint64_t _last{0};
    unsigned _next{0}, _sent{0}, _failed{0};
};
