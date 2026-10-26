// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#include "rc_input.hpp"
#include "rc_telemetry.hpp"
#include "crsf_telemetry_wire.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
uint64_t test_now = 1000000;
unsigned test_publications = 0;
input_rc_s test_last{};
int32_t test_node_id = 42;
uint64_t hrt_absolute_time() { return test_now; }
int param_find(const char *name) { assert(!strcmp(name, "UAVCAN_RC_ID")); return 0; }
int param_get(int, void *value) { *static_cast<int32_t *>(value) = test_node_id; return 0; }
class Clock : public uavcan::ISystemClock
{
public:
	uavcan::MonotonicTime getMonotonic() const override { return uavcan::MonotonicTime::fromUSec(test_now + 5000000); }
	uavcan::UtcTime getUtc() const override { return uavcan::UtcTime(); }
	void adjustUtc(uavcan::UtcDuration) override {}
};

class CanInterface : public uavcan::ICanIface
{
public:
	struct Input { uavcan::CanFrame frame; uavcan::MonotonicTime stamp; };
	std::deque<Input> input;
	std::vector<uavcan::CanFrame> output;
	int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime, uavcan::CanIOFlags) override
	{
		output.push_back(frame);
		return 1;
	}
	int16_t receive(uavcan::CanFrame &frame, uavcan::MonotonicTime &mono, uavcan::UtcTime &utc,
			uavcan::CanIOFlags &flags) override
	{
		if (input.empty()) { return 0; }

		frame = input.front().frame;
		flags = 0;
		mono = input.front().stamp;
		input.pop_front();
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
	CanInterface interfaces[1];
	uavcan::ICanIface *getIface(uint8_t index) override { return index < 1 ? &interfaces[index] : nullptr; }
	uint8_t getNumIfaces() const override { return 1; }
	int16_t select(uavcan::CanSelectMasks &masks, const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
		       uavcan::MonotonicTime) override
	{
		masks.read &= interfaces[0].input.empty() ? 0 : 1;
		masks.write &= 1;
		return 1;
	}
};

class Node : public uavcan::INode
{
public:
	CanDriver driver;
	Clock clock;
	uavcan::PoolAllocator<16384, 128> allocator;
	uavcan::Scheduler scheduler{driver, allocator, clock};
	uavcan::IPoolAllocator &getAllocator() override { return allocator; }
	uavcan::Scheduler &getScheduler() override { return scheduler; }
	const uavcan::Scheduler &getScheduler() const override { return scheduler; }
	void registerInternalFailure(const char *) override { assert(false); }

	explicit Node(unsigned id) { setNodeID(uavcan::NodeID(id)); }
};

class Sender
{
public:
	Node node;
	uavcan::Publisher<dronecan::sensors::rc::RCInput> publisher;
	explicit Sender(unsigned id) : node(id), publisher(node) {}
	void send(Node &receiver, const dronecan::sensors::rc::RCInput &msg, int64_t age = 0, bool partial = false)
	{
		auto &output = node.driver.interfaces[0].output;
		output.clear();
		assert(publisher.broadcast(msg) >= 0);
		assert(node.spinOnce() >= 0);
		assert(!output.empty());

		for (unsigned i = 0; i < output.size(); ++i) {
			if (!partial || i == output.size() - 1) {
				receiver.driver.interfaces[0].input.push_back({output[i],
						uavcan::MonotonicTime::fromUSec(test_now + 5000000 - age)});
			}
		}

		assert(receiver.spinOnce() >= 0);
	}
};

battery_status_s test_battery_status{};
sensor_gps_s test_sensor_gps{};
vehicle_attitude_s test_vehicle_attitude{};
vehicle_status_s test_vehicle_status{};
uavcan::tunnel::Targetted telemetry_packet;
unsigned telemetry_received = 0;
static void receive_telemetry(const uavcan::ReceivedDataStructure<uavcan::tunnel::Targetted> &msg)
{
	telemetry_packet = msg; ++telemetry_received;
}
static void test_telemetry()
{
	test_node_id = 42;
	Node px4(1), matek(42);
	uavcan::Subscriber<uavcan::tunnel::Targetted> sub(matek);
	assert(sub.start(&receive_telemetry) >= 0);
	UavcanRcTelemetry bridge(px4);
	assert(bridge.init() == 0);
	auto tick = [&]() {
		test_now += 100000; test_last.timestamp_last_signal = test_now;
		bridge.update();

		for (auto &frame : px4.driver.interfaces[0].output) {
			matek.driver.interfaces[0].input.push_back({frame, matek.clock.getMonotonic()});
		}

		px4.driver.interfaces[0].output.clear();
		assert(matek.spinOnce() >= 0);
	};
	test_last.rc_lost = true;
	tick(); assert(telemetry_received == 0);
	test_last.rc_lost = false; test_last.rc_failsafe = false;
	test_last.input_source = input_rc_s::RC_INPUT_SOURCE_DRONECAN;
	test_battery_status = {test_now, true, 50.1f, 12.3f, 1234.f, .75f};
	test_sensor_gps = {test_now, 3, 15, 47.0, -8.0, 500.f, 20.f, float(M_PI / 2)};
	test_vehicle_attitude.timestamp = test_now; test_vehicle_attitude.q[0] = 1.f;
	test_vehicle_status = {test_now, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION};
	const uint8_t types[] = {0x08, 0x02, 0x1e, 0x21};
	auto be = [](const uint8_t *p, unsigned n) { uint32_t v = 0; while (n--) { v = (v << 8) | *p++; } return v; };

	for (unsigned i = 0; i < 4; ++i) {
		tick(); assert(telemetry_received == i + 1);
		assert(telemetry_packet.target_node == 42 && telemetry_packet.protocol.protocol == 255);
		assert(telemetry_packet.serial_id == -1 && telemetry_packet.options == 0 && telemetry_packet.baudrate == 0);
		auto &b = telemetry_packet.buffer;
		uint8_t bytes[120];

		for (unsigned j = 0; j < b.size(); ++j) { bytes[j] = b[j]; }

		assert(noctua_crsf::valid(bytes, b.size())); assert(bytes[6] == types[i]);
		const uint8_t *payload = bytes + 7;

		if (i == 0) { assert(be(payload, 2) == 501 && be(payload + 2, 2) == 123 && be(payload + 4, 3) == 1234 && payload[7] == 75); }

		if (i == 1) { assert(int32_t(be(payload, 4)) == 470000000 && int32_t(be(payload + 4, 4)) == -80000000); assert(be(payload + 8, 2) == 720 && be(payload + 10, 2) == 9000 && be(payload + 12, 2) == 1500 && payload[14] == 15); }

		if (i == 2) { for (unsigned j = 0; j < 6; ++j) { assert(payload[j] == 0); } }

		if (i == 3) { assert(!strcmp(reinterpret_cast<const char *>(payload), "Mission")); }

		for (unsigned n = 0; n < b.size(); ++n) { assert(!noctua_crsf::valid(bytes, n)); }

		for (unsigned j = 0; j < b.size(); ++j) { bytes[j] ^= 1; assert(!noctua_crsf::valid(bytes, b.size())); bytes[j] ^= 1; }
	}

	test_now += 3000000;

	for (unsigned i = 0; i < 4; ++i) { tick(); }

	assert(telemetry_received == 4);
	test_battery_status.timestamp = test_now; test_battery_status.remaining = NAN;
	tick(); assert(telemetry_received == 4);
	test_last.rc_failsafe = true;

	for (unsigned i = 0; i < 4; ++i) { tick(); }

	assert(telemetry_received == 4);
	noctua_crsf::Frame command(0x32); command.put(1, 1); command.finish();
	assert(!noctua_crsf::valid(command.bytes, command.size));
	noctua_crsf::Frame text(0x21);

	for (unsigned i = 0; i < 17; ++i) { text.put('x', 1); } text.put(0, 1); text.finish();

	assert(!noctua_crsf::valid(text.bytes, text.size));
	test_node_id = 0; UavcanRcTelemetry disabled(px4);
	assert(disabled.init() == 0); disabled.update(); assert(px4.driver.interfaces[0].output.empty());
	bridge.print_status();
	puts("PASS: production telemetry, real DroneCAN, units, frame integrity, stale-data and failsafe suppression");
}

using Rc = dronecan::sensors::rc::RCInput;
static Rc channels(unsigned count = 16)
{
	Rc msg;
	msg.status = Rc::STATUS_QUALITY_VALID;
	msg.quality = 255;
	msg.id = 0;

	for (unsigned i = 0; i < count; ++i) { msg.rcin.push_back(1000 + i * 20); }

	return msg;
}
int main()
{
	Node receiver(1);
	Sender sender(42), other(43);
	{
		UavcanRcInputBridge bridge(receiver);
		assert(bridge.init() >= 0);
		bridge.update();
		assert(test_publications == 0);
		auto msg = channels();
		other.send(receiver, msg);
		sender.send(receiver, channels(3));
		assert(test_publications == 0);
		test_now += 20000;
		sender.send(receiver, msg, 10000);
		assert(test_publications == 1);
		assert(test_last.input_source == input_rc_s::RC_INPUT_SOURCE_DRONECAN);
		assert(test_last.channel_count == 16 && test_last.values[15] == 1300);
		assert(test_last.values[16] == 0 && test_last.values[17] == 0);
		assert(!test_last.rc_lost && !test_last.rc_failsafe && test_last.rssi == 100);
		assert(test_last.timestamp_last_signal == test_now - 10000);
		assert(test_last.link_quality == -1 && std::isnan(test_last.rssi_dbm));
		const auto healthy_time = test_last.timestamp_last_signal;
		msg.id = 1;
		sender.send(receiver, msg);
		other.send(receiver, msg);
		assert(test_publications == 1);
		test_now = healthy_time + 99999;
		bridge.update();
		assert(!test_last.rc_lost);
		++test_now;
		bridge.update();
		assert(test_last.rc_lost && test_last.timestamp_last_signal == healthy_time);
		assert(test_publications == 2);
		bridge.update();
		assert(test_publications == 2);
		msg = channels(32);
		sender.send(receiver, msg);
		assert(!test_last.rc_lost && test_last.channel_count == 18 && test_last.values[17] == 1340);
		const auto last_healthy = test_last.timestamp_last_signal;
		test_now += 20000;
		msg.status |= Rc::STATUS_FAILSAFE;
		sender.send(receiver, msg);
		assert(test_last.rc_lost && test_last.rc_failsafe);
		assert(test_last.timestamp_last_signal == last_healthy);
		test_now += 20000;
		msg.rcin.clear();
		sender.send(receiver, msg);
		assert(test_last.rc_failsafe && test_last.channel_count == 0);
		assert(test_last.timestamp_last_signal == last_healthy && test_last.values[0] == 0);
		sender.send(receiver, channels());
		assert(!test_last.rc_lost && !test_last.rc_failsafe);
		sender.send(receiver, channels(3));
		assert(test_last.rc_lost);
		const auto publications = test_publications;
		test_now += 120000;
		sender.send(receiver, channels(), 100000);
		sender.send(receiver, channels(), -1);
		assert(test_publications == publications);
		test_now += 20000;
		sender.send(receiver, channels());
		assert(!test_last.rc_lost);
		const auto accepted = test_last.timestamp_last_signal;
		test_now += 100000;
		sender.send(receiver, channels(), 0, true);
		bridge.update();
		assert(test_last.rc_lost && test_last.timestamp_last_signal == accepted);
		bridge.print_status();
	}
	test_node_id = 0;
	{
		UavcanRcInputBridge bridge(receiver);
		assert(bridge.init() >= 0);
		auto msg = channels();
		msg.status = Rc::STATUS_FAILSAFE;
		const auto count = test_publications;
		other.send(receiver, msg);
		assert(test_publications == count);
		msg.status = Rc::STATUS_QUALITY_VALID;

		for (unsigned quality : {0u, 127u, 255u}) {
			msg.quality = quality;
			sender.send(receiver, msg);
			assert(test_last.rssi == (quality == 0 ? 0 : quality == 127 ? 50 : 100));
		}
		msg.status = 1 | 4;
		msg.quality = 0x80 | 97;
		sender.send(receiver, msg);
		assert(test_last.link_quality == 97 && test_last.rssi == -1);
		msg.quality = 127;
		sender.send(receiver, msg);
		assert(test_last.link_quality == -1);
		msg.status = 1 | 8;
		msg.quality = 85;
		sender.send(receiver, msg);
		assert(test_last.rssi_dbm == -85.f && test_last.rssi == -1);
		msg.quality = 0;
		sender.send(receiver, msg);
		assert(std::isnan(test_last.rssi_dbm));

		for (unsigned status : {0u, 13u, 17u, 29u}) {
			msg.status = status;
			sender.send(receiver, msg);
			assert(test_last.rssi == -1 && test_last.link_quality == -1 && std::isnan(test_last.rssi_dbm));
		}
		test_now += 100000;
		bridge.update();
		assert(test_last.rc_lost);
		const auto count_lost = test_publications;
		other.send(receiver, channels());
		assert(test_publications == count_lost && test_last.rc_lost);
		sender.send(receiver, channels());
		assert(!test_last.rc_lost);
	}
	test_telemetry();
	puts("PASS: real DroneCAN transfers, channel bounds, source locking, quality, failsafe, stale/partial transfers and timeout recovery");
}
