// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#include "../Protocol.hpp"

#include <array>
#include <cassert>
#include <cstdio>

using ze07_h2::Parser;
using Frame = std::array<uint8_t, ze07_h2::PACKET_SIZE>;

static void checksum(Frame &frame)
{
	uint8_t sum = 0;

	for (size_t i = 1; i < 8; ++i) { sum += frame[i]; }

	frame[8] = uint8_t(0 - sum);
}

static Frame measurement(uint16_t ppm)
{
	Frame frame{0xff, 6, 3, 0, uint8_t(ppm >> 8), uint8_t(ppm), 0x75, 0x30, 0};
	checksum(frame);
	return frame;
}

static unsigned feed(Parser &parser, const Frame &frame, uint16_t &ppm)
{
	unsigned count = 0;

	for (auto byte : frame) { count += parser.feed(byte, ppm) == Parser::Result::Measurement; }

	return count;
}

int main()
{
	uint16_t ppm = 65535;
	Parser parser;
	// Literal manufacturer's table 4 vector, independent of checksum helper.
	assert(feed(parser, {0xff, 6, 3, 0, 0, 0x25, 0x75, 0x30, 0x2d}, ppm) == 1 && ppm == 37);

	// Exhaustive representable range, including zero, full scale, embedded FF,
	// and checksum FF. Byte-wise input also exercises fragmented UART reads.
	for (unsigned value = 0; value <= 30000; ++value) {
		assert(feed(parser, measurement(value), ppm) == 1 && ppm == value);
	}

	for (unsigned value : {30001u, 65535u}) {
		parser.reset();
		assert(feed(parser, measurement(value), ppm) == 0);
	}

	// Correct checksum alone is not enough: reject gas/unit/scaling/range errors.
	for (size_t field : {1u, 2u, 3u, 6u, 7u}) {
		parser.reset();
		auto bad = measurement(100);
		bad[field] ^= 1;
		checksum(bad);
		ppm = 65535;
		assert(feed(parser, bad, ppm) == 0 && ppm == 65535);
		assert(feed(parser, measurement(500), ppm) == 1 && ppm == 500);
	}

	// Every single-bit corruption must fail, then recover at the next frame.
	for (size_t offset = 0; offset < 9; ++offset) {
		for (unsigned bit = 0; bit < 8; ++bit) {
			parser.reset();
			auto bad = measurement(255);
			bad[offset] ^= 1u << bit;
			assert(feed(parser, bad, ppm) == 0);
			assert(feed(parser, measurement(1000), ppm) == 1 && ppm == 1000);
		}
	}

	// Recover a new header overlapping a truncated frame (all split points).
	for (size_t length = 1; length < 9; ++length) {
		parser.reset();
		auto partial = measurement(255);

		for (size_t i = 0; i < length; ++i) { parser.feed(partial[i], ppm); }

		assert(feed(parser, measurement(1234), ppm) == 1 && ppm == 1234);
	}

	// Long noise, repeated headers, complete frames back-to-back.
	parser.reset();

	for (unsigned i = 0; i < 10000; ++i) { parser.feed(0xff, ppm); }

	assert(feed(parser, measurement(250), ppm) == 1 && ppm == 250);
	assert(feed(parser, measurement(500), ppm) == 1 && ppm == 500);
	parser.reset();
	// Q&A reply and echoed mode-setting command are not gas measurements.
	assert(feed(parser, {0xff, 0x86, 0, 0x20, 0, 0, 0, 0x20, 0x3a}, ppm) == 0);
	parser.reset();
	assert(feed(parser, {0xff, 1, 0x78, 0x40, 0, 0, 0, 0, 0x47}, ppm) == 0);

	// Explicit reset models an inter-byte timeout or UART reopen.
	parser.feed(0xff, ppm);
	parser.feed(6, ppm);
	parser.reset();
	assert(feed(parser, measurement(750), ppm) == 1 && ppm == 750);
	puts("ZE07-H2 protocol tests passed");
}
