// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "Bridge.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>

uint64_t test_now = 1000000;
int32_t test_enabled = 1;
int32_t test_interface = 2;
int32_t test_timeout = 500;
unsigned test_publications = 0;
ie_fuelcell_can_status_s test_last{};

uint64_t hrt_absolute_time() { return test_now; }
int param_find(const char *name)
{
	if (!strcmp(name, "IEFC_CAN_EN")) { return 0; }
	if (!strcmp(name, "IEFC_CAN_IFACE")) { return 1; }
	if (!strcmp(name, "IEFC_CAN_TOUT")) { return 2; }
	assert(false);
	return -1;
}
int param_get(int id, void *value)
{
	*static_cast<int32_t *>(value) = id == 0 ? test_enabled : id == 1 ? test_interface : test_timeout;
	return 0;
}

static const uint8_t Golden[8] = {0x52, 0x96, 0x1f, 0x11, 0x18, 0x82, 0x7b, 0xd5};

static void close_to(float actual, float expected)
{
	assert(std::fabs(actual - expected) < 0.001f);
}

static void test_protocol()
{
	ie_fuelcell::Status status{};
	assert(ie_fuelcell::decode(0x400, true, false, false, Golden, 8, status));
	close_to(status.tank_pressure_bar, 300.f);
	close_to(status.battery_voltage_v, 49.7f);
	close_to(status.output_power_w, 700.f);
	close_to(status.stack_power_w, 650.f);
	close_to(status.battery_power_w, -50.f);
	assert(status.counter == 5 && status.state == 2 && status.error == 21);

	// Actual MCU v3.63 capture. Voltage agrees with the independently logged
	// ~46.2 V battery; the old LSB-first decoder incorrectly produced 0.7 V.
	const uint8_t captured[8] = {0x80, 0x00, 0x1c, 0xe0, 0x00, 0x01, 0x3e, 0x60};
	assert(ie_fuelcell::decode(0x400, true, false, false, captured, 8, status));
	close_to(status.battery_voltage_v, 46.2f);
	close_to(status.tank_pressure_bar, 0.f);
	close_to(status.output_power_w, 0.f);
	assert(status.counter == 8 && status.state == 0 && status.error == 32);
	// These only verify the manual's conversions, not physical correctness
	// on MCU v3.63: the battery-power discrepancy remains under investigation.
	close_to(status.stack_power_w, 5.f);
	close_to(status.battery_power_w, -2510.f);

	// Verify every wire bit independently, including field boundaries and the
	// sign/bias of battery power. The reference uses the manual's bit offsets.
	const unsigned starts[] = {8, 18, 28, 38, 48};
	const float scales[] = {0.5f, 0.1f, 10.f, 5.f, 10.f};

	for (unsigned bit = 0; bit < 64; ++bit) {
		uint8_t bytes[8]{};
		bytes[bit / 8] = 1u << (7 - bit % 8);
		assert(ie_fuelcell::decode(0x400, true, false, false, bytes, 8, status));
		const float fields[] = {status.tank_pressure_bar, status.battery_voltage_v, status.output_power_w,
					status.stack_power_w, status.battery_power_w};

		for (unsigned field = 0; field < 5; ++field) {
			float expected = field == 4 ? -5000.f : 0.f;

			if (bit >= starts[field] && bit < starts[field] + 10) {
				expected += float(1u << (starts[field] + 9 - bit)) * scales[field];
			}

			close_to(fields[field], expected);
		}

		assert(status.counter == (bit < 4 ? 1u << (3 - bit) : 0));
		assert(status.state == (bit >= 4 && bit < 8 ? 1u << (7 - bit) : 0));
		assert(status.error == (bit >= 58 ? 1u << (63 - bit) : 0));
	}

	uint8_t maximum[8];
	memset(maximum, 0xff, sizeof(maximum));
	assert(ie_fuelcell::decode(0x400, true, false, false, maximum, 8, status));
	close_to(status.tank_pressure_bar, 511.5f);
	close_to(status.battery_voltage_v, 102.3f);
	close_to(status.output_power_w, 10230.f);
	close_to(status.stack_power_w, 5115.f);
	close_to(status.battery_power_w, 5230.f);
	assert(status.counter == 15 && status.state == 15 && status.error == 63);

	// The CAN battery power is offset-binary, not a signed 10-bit integer.
	for (unsigned raw = 0; raw < 1024; ++raw) {
		uint8_t bytes[8]{};
		bytes[6] = raw >> 2;
		bytes[7] = (raw & 3u) << 6;
		assert(ie_fuelcell::decode(0x400, true, false, false, bytes, 8, status));
		close_to(status.battery_power_w, int(raw) * 10.f - 5000.f);
	}

	assert(!ie_fuelcell::decode(0x401, true, false, false, Golden, 8, status));
	assert(!ie_fuelcell::decode(0x400, false, false, false, Golden, 8, status));
	assert(!ie_fuelcell::decode(0x400, true, true, false, Golden, 8, status));
	assert(!ie_fuelcell::decode(0x400, true, false, true, Golden, 8, status));
	assert(!ie_fuelcell::decode(0x400, true, false, false, nullptr, 8, status));

	for (unsigned length = 0; length < 16; ++length) {
		if (length != 8) {
			assert(!ie_fuelcell::decode(0x400, true, false, false, Golden, length, status));
		}
	}
}

class Clock : public uavcan::ISystemClock
{
public:
	uavcan::MonotonicTime getMonotonic() const override { return uavcan::MonotonicTime::fromUSec(test_now + 100); }
	uavcan::UtcTime getUtc() const override { return uavcan::UtcTime(); }
	void adjustUtc(uavcan::UtcDuration) override {}
};

class CanInterface : public uavcan::ICanIface
{
public:
	struct Input { uavcan::CanFrame frame; uavcan::CanIOFlags flags; };
	std::deque<Input> input;
	unsigned transmissions = 0;
	int16_t send(const uavcan::CanFrame &, uavcan::MonotonicTime, uavcan::CanIOFlags) override
	{
		++transmissions;
		return 1;
	}
	int16_t receive(uavcan::CanFrame &frame, uavcan::MonotonicTime &mono, uavcan::UtcTime &utc,
			uavcan::CanIOFlags &flags) override
	{
		if (input.empty()) { return 0; }
		frame = input.front().frame;
		flags = input.front().flags;
		input.pop_front();
		mono = uavcan::MonotonicTime::fromUSec(test_now + 100);
		utc = uavcan::UtcTime();
		return 1;
	}
	int16_t configureFilters(const uavcan::CanFilterConfig *, uint16_t) override { return 0; }
	uint16_t getNumFilters() const override { return 0; }
	uint64_t getErrorCount() const override { return 0; }
};

class CanDriver : public uavcan::ICanDriver
{
public:
	CanInterface interfaces[2];
	uavcan::ICanIface *getIface(uint8_t index) override { return index < 2 ? &interfaces[index] : nullptr; }
	uint8_t getNumIfaces() const override { return 2; }
	int16_t select(uavcan::CanSelectMasks &masks, const uavcan::CanFrame *(&)[uavcan::MaxCanIfaces],
		       uavcan::MonotonicTime) override
	{
		masks.read = (interfaces[0].input.empty() ? 0 : 1) | (interfaces[1].input.empty() ? 0 : 2);
		masks.write = 3;
		return 2;
	}
};

class Node : public uavcan::INode
{
public:
	CanDriver driver;
	Clock clock;
	uavcan::PoolAllocator<8192, 48> allocator;
	uavcan::Scheduler scheduler{driver, allocator, clock};
	uavcan::IPoolAllocator &getAllocator() override { return allocator; }
	uavcan::Scheduler &getScheduler() override { return scheduler; }
	const uavcan::Scheduler &getScheduler() const override { return scheduler; }
	void registerInternalFailure(const char *) override { assert(false); }

	void receive(unsigned interface, uint8_t counter, uint32_t id = uavcan::CanFrame::FlagEFF | 0x400,
		     uint8_t length = 8)
	{
		uavcan::CanFrame frame(id, Golden, length);
		frame.data[0] = (frame.data[0] & 0x0f) | (counter << 4);
		driver.interfaces[interface].input.push_back({frame, 0});
		assert(spinOnce() >= 0);
	}
};

static void test_bridge()
{
	Node node;
	{
		IeFuelcellCanBridge bridge(node);
		bridge.init();
		assert(node.getDispatcher().getRxFrameListener() == &bridge);
		assert(!test_last.connected && test_last.timestamp_sample == 0);
		assert(std::isnan(test_last.tank_pressure_bar));
		assert(test_publications == 1);

		// Wrong port, standard frame, RTR, error, unrelated ID and bad DLC.
		node.receive(0, 5);
		node.receive(1, 5, 0x400);
		node.receive(1, 5, uavcan::CanFrame::FlagEFF | uavcan::CanFrame::FlagRTR | 0x400);
		node.receive(1, 5, uavcan::CanFrame::FlagEFF | uavcan::CanFrame::FlagERR | 0x400);
		node.receive(1, 5, uavcan::CanFrame::FlagEFF | 0x401);
		node.receive(1, 5, uavcan::CanFrame::FlagEFF | 0x400, 7);
		assert(test_publications == 1);

		// Loopback is explicitly rejected by the bridge (do not feed arbitrary
		// non-DroneCAN loopbacks into libuavcan's loopback parser).
		uavcan::CanRxFrame loopback;
		loopback.id = uavcan::CanFrame::FlagEFF | 0x400;
		loopback.iface_index = 1;
		loopback.dlc = 8;
		memcpy(loopback.data, Golden, 8);
		bridge.handleRxFrame(loopback, uavcan::CanIOFlagLoopback);
		assert(test_publications == 1);

		node.receive(1, 15);
		assert(test_last.connected && test_last.interface == 2 && test_last.received_frames == 1);
		assert(test_last.timestamp_sample == test_now);
		close_to(test_last.battery_power_w, -50.f);
		test_now += 100000;
		node.receive(1, 0); // counter wrap
		assert(test_last.received_frames == 2);
		const unsigned publications = test_publications;
		test_now += 499999;
		node.receive(1, 0); // even an identical frame proves link activity
		bridge.update();
		assert(test_publications == publications + 1 && test_last.timestamp_sample == test_now);
		assert(test_last.received_frames == 3 && test_last.duplicate_frames == 1);
		assert(test_last.connected);
		assert(test_last.raw_data[0] == 0x02);
		assert(memcmp(test_last.raw_data + 1, Golden + 1, 7) == 0);

		// Same counter with a changed measurement must update that measurement.
		test_now += 100000;
		uavcan::CanFrame changed(uavcan::CanFrame::FlagEFF | 0x400, Golden, 8);
		changed.data[0] = 0x02;
		changed.data[6] = 0x7e; // raw 505: battery power -50 W -> +50 W
		changed.data[7] = 0x55;
		node.driver.interfaces[1].input.push_back({changed, 0});
		assert(node.spinOnce() >= 0);
		close_to(test_last.battery_power_w, 50.f);
		assert(test_last.received_frames == 4 && test_last.duplicate_frames == 2);
		assert(memcmp(test_last.raw_data, changed.data, 8) == 0);
		const uint64_t accepted = test_now;

		// Actual silence times out; rejected input must not keep the link alive.
		test_now += 499999;
		node.receive(0, 1);
		node.receive(1, 1, 0x400);
		bridge.update();
		assert(test_last.connected);
		++test_now;
		bridge.update();
		assert(!test_last.connected && std::isnan(test_last.battery_voltage_v));
		assert(test_last.timestamp_sample == accepted && test_last.duplicate_frames == 2);
		assert(test_publications == publications + 3);
		bridge.update();
		assert(test_publications == publications + 3); // one timeout notification
		node.receive(1, 0);
		assert(test_last.connected); // reconnect can have the same counter
		close_to(test_last.battery_power_w, -50.f);
		node.receive(1, 7); // gaps are allowed
		assert(test_last.connected && test_last.received_frames == 6);
		assert(test_last.duplicate_frames == 3);
		assert(node.driver.interfaces[0].transmissions == 0 && node.driver.interfaces[1].transmissions == 0);
		bridge.print_status();
	}
	assert(node.getDispatcher().getRxFrameListener() == nullptr);

	// Disabled/invalid configuration must leave the CAN dispatcher untouched.
	test_enabled = 0;
	{
		IeFuelcellCanBridge bridge(node);
		bridge.init();
		assert(node.getDispatcher().getRxFrameListener() == nullptr);
	}
	test_enabled = 1;
	test_interface = 3;
	{
		IeFuelcellCanBridge bridge(node);
		bridge.init();
		assert(node.getDispatcher().getRxFrameListener() == nullptr);
	}
	test_interface = 1;
	test_timeout = 0;
	{
		IeFuelcellCanBridge bridge(node);
		bridge.init();
		assert(node.getDispatcher().getRxFrameListener() == nullptr);
	}
	test_timeout = 500;
	{
		IeFuelcellCanBridge first(node);
		first.init();
		{
			IeFuelcellCanBridge second(node);
			second.init();
			assert(node.getDispatcher().getRxFrameListener() == &first);
		}
		assert(node.getDispatcher().getRxFrameListener() == &first);
	}
	assert(node.getDispatcher().getRxFrameListener() == nullptr);
}

int main()
{
	test_protocol();
	test_bridge();
	puts("PASS: S800 decoding, raw libuavcan dispatch, interface filtering, freshness and lifecycle");
}
