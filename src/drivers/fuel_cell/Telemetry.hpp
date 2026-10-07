// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <uORB/topics/fuel_cell.h>
#include <uORB/topics/fuel_cell_can.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <climits>

namespace fuelcell
{
inline fuel_cell_s unavailable(uint8_t source, uint64_t now, uint64_t sample = 0)
{
	fuel_cell_s out{};
	out.timestamp = now;
	out.timestamp_sample = sample;
	out.source = source;
	out.tankpressure = out.regpressure = out.voltage = NAN;
	out.outputpower = out.spmpower = out.battpower = out.tank_pressure_bar = NAN;
	out.psustate = out.mainerror = out.suberror = out.raw_can_error = -1;
	return out;
}

inline bool fresh(uint64_t sample, uint64_t now, uint64_t timeout)
{
	return sample != 0 && now >= sample && now - sample < timeout;
}

inline fuel_cell_s from_can(const fuel_cell_can_s &can, uint8_t source, uint64_t now, uint64_t timeout)
{
	auto out = unavailable(source, now, can.timestamp_sample);

	if (source < 2 || source > 3 || can.interface != source - 1 || !can.connected
	    || !fresh(can.timestamp_sample, now, timeout)) {
		return out;
	}

	out.connected = true;
	out.voltage = can.battery_voltage_v;
	out.tank_pressure_bar = can.tank_pressure_bar;
	out.psustate = can.state;
	out.raw_can_error = can.error_code;
	// CAN powers disagree with UART on MCU v3.63. Keep unverified values
	// out of the shared topic; retain them in internal CAN diagnostics.
	return out;
}

inline bool number(const char *text, float &value)
{
	char *end = nullptr;
	errno = 0;
	value = strtof(text, &end);
	return end != text && *end == '\0' && errno != ERANGE && std::isfinite(value);
}

inline bool integer(const char *text, int32_t &value)
{
	char *end = nullptr;
	errno = 0;
	const long parsed = strtol(text, &end, 10);

	if (end == text || *end != '\0' || errno == ERANGE || parsed < INT32_MIN || parsed > INT32_MAX) {
		return false;
	}

	value = static_cast<int32_t>(parsed);
	return true;
}

// UART customer formats 4-6 share the first ten fields and final checksum.
// Keep empty fields (notably unit-in-fault/info); strtok would shift them.
inline bool parse(char *line, fuel_cell_s &out, uint64_t now)
{
	const size_t size = strlen(line);

	if (size < 3 || line[0] != '<' || line[size - 1] != '>') { return false; }

	char *last_comma = strrchr(line, ',');

	if (!last_comma) { return false; }

	unsigned sum = 0;

	for (const char *p = line; p <= last_comma; ++p) { sum += static_cast<unsigned char>(*p); }

	line[size - 1] = '\0';
	int32_t checksum = -1;

	if (!integer(last_comma + 1, checksum) || checksum != int(255 - (sum & 255))) { return false; }

	char *fields[20]{};
	unsigned count = 0;
	fields[count++] = line + 1;

	for (char *p = line + 1; *p; ++p) {
		if (*p == ',') {
			*p = '\0';

			if (count == 20) { return false; }

			fields[count++] = p + 1;
		}
	}

	if (count < 12) { return false; }

	auto sample = unavailable(1, now, now);

	if (!number(fields[0], sample.tankpressure) || !number(fields[1], sample.regpressure)
	    || !number(fields[2], sample.voltage) || !number(fields[3], sample.outputpower)
	    || !number(fields[4], sample.spmpower) || !number(fields[6], sample.battpower)
	    || !integer(fields[7], sample.psustate) || !integer(fields[8], sample.mainerror)
	    || !integer(fields[9], sample.suberror)) { return false; }

	sample.connected = true;
	out = sample;
	return true;
}

class SerialParser
{
public:
	bool feed(char c, fuel_cell_s &sample, uint64_t now)
	{
		// A new start marker also resynchronizes after a truncated frame.
		if (c == '<' || c == '[') { _length = 0; _discard = false; }

		if (c == '\r' || c == '\n') {
			_line[_length] = '\0';
			const bool valid = !_discard && _length && parse(_line, sample, now);
			_length = 0;
			_discard = false;
			return valid;
		}

		if (!_discard && _length < sizeof(_line) - 1) { _line[_length++] = c; }
		else { _discard = true; }

		return false;
	}
private:
	char _line[256]{};
	size_t _length{0};
	bool _discard{false};
};
} // namespace fuelcell
