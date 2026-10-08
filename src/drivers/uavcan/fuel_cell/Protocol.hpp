// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cstddef>
#include <cstdint>

namespace fuelcell_can
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

	// Fields follow the table from left to right, most significant bit first.
	// The table's "Bit 0" is the first (MSB) bit on the wire, not the C bit 0.
	// Avoid packed structs, unaligned loads and C++ bitfield layout assumptions.
	status.counter = bytes[0] >> 4;
	status.state = bytes[0] & 0x0fu;
	status.tank_pressure_bar = ((bytes[1] << 2) | (bytes[2] >> 6)) * 0.5f;
	status.battery_voltage_v = (((bytes[2] & 0x3fu) << 4) | (bytes[3] >> 4)) * 0.1f;
	status.output_power_w = (((bytes[3] & 0x0fu) << 6) | (bytes[4] >> 2)) * 10.f;
	status.stack_power_w = (((bytes[4] & 0x03u) << 8) | bytes[5]) * 5.f;
	// Retain the manual's -5000..5230 W range; MCU v3.63 scaling still needs
	// comparison with simultaneous manufacturer telemetry (see README).
	status.battery_power_w = ((bytes[6] << 2) | (bytes[7] >> 6)) * 10.f - 5000.f;
	status.error = bytes[7] & 0x3fu;
	return true;
}

// Link freshness comes from frame reception. The manual does not promise an
// autonomously advancing TX counter; on hardware it may remain unchanged.
// Track repeated counters for diagnostics without discarding their payloads.
class Freshness
{
public:
	bool observe(uint8_t counter, uint64_t now)
	{
		const bool changed = !_received || counter != _counter;
		_received = true;
		_counter = counter;
		_last_update = now;
		return changed;
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

} // namespace fuelcell_can
