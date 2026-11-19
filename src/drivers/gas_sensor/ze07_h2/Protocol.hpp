// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ze07_h2
{

// Winsen ZE07-H2 manual v1.0, tables 4 and 6. Only initiative upload
// packets identify the gas and units, so Q&A replies are not accepted.
static constexpr uint8_t START_UPLOAD[] {0xff, 0x01, 0x78, 0x40, 0, 0, 0, 0, 0x47};
static constexpr size_t PACKET_SIZE = 9;

class Parser
{
public:
	enum class Result { Incomplete, Invalid, Measurement };

	void reset() { _length = 0; }

	Result feed(uint8_t byte, uint16_t &ppm)
	{
		if (_length == 0 && byte != 0xff) { return Result::Incomplete; }

		_buffer[_length++] = byte;

		if (_length < PACKET_SIZE) { return Result::Incomplete; }

		uint8_t sum = 0;

		for (size_t i = 1; i < PACKET_SIZE; ++i) { sum += _buffer[i]; }

		const uint16_t concentration = (uint16_t(_buffer[4]) << 8) | _buffer[5];
		const uint16_t full_scale = (uint16_t(_buffer[6]) << 8) | _buffer[7];

		if (sum == 0 && _buffer[1] == 0x06 && _buffer[2] == 0x03 && _buffer[3] == 0
		    && full_scale == 30000 && concentration <= full_scale) {
			ppm = concentration;
			reset();
			return Result::Measurement;
		}

		// Preserve a possible header within a corrupt/partial packet, including
		// 0xff inside its payload. Never discard the start of the next frame.
		size_t next = 1;

		while (next < PACKET_SIZE && _buffer[next] != 0xff) { ++next; }

		_length = PACKET_SIZE - next;
		memmove(_buffer, _buffer + next, _length);
		return Result::Invalid;
	}

private:
	uint8_t _buffer[PACKET_SIZE] {};
	size_t _length{0};
};

} // namespace ze07_h2
