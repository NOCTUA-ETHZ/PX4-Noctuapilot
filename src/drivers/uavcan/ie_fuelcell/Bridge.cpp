// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "Bridge.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

#include <cmath>
#include <cstdio>

IeFuelcellCanBridge::~IeFuelcellCanBridge()
{
	if (_node.getDispatcher().getRxFrameListener() == this) {
		_node.removeRxFrameListener();
	}
}

void IeFuelcellCanBridge::init()
{
	int32_t enabled = 0;
	param_get(param_find("IEFC_CAN_EN"), &enabled);

	if (enabled == 0) {
		return;
	}

	int32_t interface = 1;
	int32_t timeout_ms = 500;
	param_get(param_find("IEFC_CAN_IFACE"), &interface);
	param_get(param_find("IEFC_CAN_TOUT"), &timeout_ms);
	const unsigned interfaces = _node.getDispatcher().getCanIOManager().getCanDriver().getNumIfaces();

	if (interface < 1 || static_cast<unsigned>(interface) > interfaces || timeout_ms < 200 || timeout_ms > 5000) {
		PX4_ERR("IE fuel cell: invalid CAN interface or timeout");
		return;
	}

	// libuavcan exposes one listener slot. Never silently replace its owner.
	if (_node.getDispatcher().getRxFrameListener() != nullptr) {
		PX4_ERR("IE fuel cell: CAN frame listener already in use");
		return;
	}

	_status.interface = interface;
	_timeout_us = static_cast<uint64_t>(timeout_ms) * 1000;
	_node.installRxFrameListener(this);
	_enabled = true;
	invalidate(hrt_absolute_time());
	PX4_INFO("IE fuel cell: CAN%d format 2 telemetry enabled", static_cast<int>(interface));
}

void IeFuelcellCanBridge::handleRxFrame(const uavcan::CanRxFrame &frame, uavcan::CanIOFlags flags)
{
	if (!_enabled || (flags & uavcan::CanIOFlagLoopback) || frame.iface_index != _status.interface - 1) {
		return;
	}

	if ((frame.id & uavcan::CanFrame::MaskExtID) != ie_fuelcell::StatusId) {
		return;
	}

	ie_fuelcell::Status decoded{};

	if (!ie_fuelcell::decode(frame.id & uavcan::CanFrame::MaskExtID, frame.isExtended(),
				frame.isRemoteTransmissionRequest(), frame.isErrorFrame(), frame.data, frame.dlc, decoded)) {
		++_invalid_frames;
		return;
	}

	// libuavcan's monotonic clock has a different origin from PX4's HRT on
	// some boards. Use HRT here rather than copying frame.ts_monotonic.
	const uint64_t now = hrt_absolute_time();

	if (!_freshness.accept(decoded.counter, now)) {
		++_status.duplicate_frames;
		return;
	}

	_status.timestamp = now;
	_status.timestamp_sample = now;
	_status.connected = true;
	_status.tank_pressure_bar = decoded.tank_pressure_bar;
	_status.battery_voltage_v = decoded.battery_voltage_v;
	_status.output_power_w = decoded.output_power_w;
	_status.stack_power_w = decoded.stack_power_w;
	_status.battery_power_w = decoded.battery_power_w;
	_status.counter = decoded.counter;
	_status.state = decoded.state;
	_status.error_code = decoded.error;
	++_status.received_frames;
	_status_pub.publish(_status);
}

void IeFuelcellCanBridge::invalidate(uint64_t now)
{
	_status.timestamp = now;
	_status.connected = false;
	_status.tank_pressure_bar = NAN;
	_status.battery_voltage_v = NAN;
	_status.output_power_w = NAN;
	_status.stack_power_w = NAN;
	_status.battery_power_w = NAN;
	_status_pub.publish(_status);
}

void IeFuelcellCanBridge::update()
{
	if (_enabled && _status.connected) {
		const uint64_t now = hrt_absolute_time();

		if (!_freshness.fresh(now, _timeout_us)) {
			invalidate(now);
		}
	}
}

void IeFuelcellCanBridge::print_status() const
{
	if (!_enabled) {
		printf("IE fuel cell CAN: disabled\n");
		return;
	}

	printf("IE fuel cell CAN%u: %s, format 2, receive only\n", unsigned(_status.interface),
	       _status.connected ? "connected" : "waiting/stale");
	printf("  accepted: %lu, duplicate: %lu, invalid: %lu\n", (unsigned long)_status.received_frames,
	       (unsigned long)_status.duplicate_frames, (unsigned long)_invalid_frames);

	if (_status.timestamp_sample != 0) {
		printf("  sample age: %llu ms, counter: %u, state: %u, raw error: %u\n",
		       (unsigned long long)((hrt_absolute_time() - _status.timestamp_sample) / 1000),
		       unsigned(_status.counter), unsigned(_status.state), unsigned(_status.error_code));
	}
}
