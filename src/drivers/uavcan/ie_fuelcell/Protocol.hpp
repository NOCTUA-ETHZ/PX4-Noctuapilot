// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cstddef>
#include <cstdint>

namespace ie_fuelcell
{

// IE-SOAR S800 User Manual V1.2, section 9.2.2, customer CAN format 2.
// This is an extended CAN identifier, even though its value fits in 11 bits.
static constexpr uint32_t StatusId = 0x400;

struct Status {
	float tank_pressure_bar;
	float battery_voltage_v;
	float output_power_w;
	float stack_power_w;
	float battery_power_w;
	uint8_t counter;
	uint8_t state;
	uint8_t error;
};

inline bool decode(uint32_t id, bool extended, bool remote, bool error, const uint8_t *bytes, size_t size,
		   Status &status)
{
	if (id != StatusId || !extended || remote || error || bytes == nullptr || size != 8) {
		return false;
	}

	// Fields span adjacent bytes, least significant bits first. Avoid packed
	// structs, unaligned loads and implementation-defined C++ bitfield layout.
	status.counter = bytes[0] & 0x0f;
	status.state = bytes[0] >> 4;
	status.tank_pressure_bar = (bytes[1] | ((bytes[2] & 0x03u) << 8)) * 0.5f;
	status.battery_voltage_v = ((bytes[2] >> 2) | ((bytes[3] & 0x0fu) << 6)) * 0.1f;
	status.output_power_w = ((bytes[3] >> 4) | ((bytes[4] & 0x3fu) << 4)) * 10.f;
	status.stack_power_w = ((bytes[4] >> 6) | (bytes[5] << 2)) * 5.f;
	// The documented range is -5000..5230 W: subtract the 5000 W bias.
	status.battery_power_w = (bytes[6] | ((bytes[7] & 0x03u) << 8)) * 10.f - 5000.f;
	status.error = bytes[7] >> 2;
	return true;
}

// A repeated counter must not keep old telemetry alive. Accept jumps and
// wraparound: frames may be lost, and the FCPM may restart independently.
class Freshness
{
public:
	bool accept(uint8_t counter, uint64_t now)
	{
		if (_received && counter == _counter) {
			return false;
		}

		_received = true;
		_counter = counter;
		_last_update = now;
		return true;
	}

	bool fresh(uint64_t now, uint64_t timeout) const
	{
		return _received && now >= _last_update && now - _last_update < timeout;
	}

private:
	bool _received{false};
	uint8_t _counter{0};
	uint64_t _last_update{0};
};

} // namespace ie_fuelcell
