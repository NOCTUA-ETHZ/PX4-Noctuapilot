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
int32_t test_backup_id = 0;
int param_find(const char *name) {
	if (!strcmp(name, "UAVCAN_RC_ID")) { return 0; }
	assert(!strcmp(name, "UAVCAN_RC_BK_ID")); return 1;
}
int param_get(int param, void *value) { *static_cast<int32_t *>(value) = param == 0 ? test_node_id : test_backup_id; return 0; }
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

airspeed_validated_s test_airspeed_validated{};
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
	test_airspeed_validated = {test_now, 15.f, 1};
	const uint8_t types[] = {0x08, 0x02, 0x1e, 0x21, 0x0a};
	auto be = [](const uint8_t *p, unsigned n) { uint32_t v = 0; while (n--) { v = (v << 8) | *p++; } return v; };

	for (unsigned i = 0; i < 5; ++i) {
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

		if (i == 4) { assert(be(payload, 2) == 540 && b.size() == 10); }

		for (unsigned n = 0; n < b.size(); ++n) { assert(!noctua_crsf::valid(bytes, n)); }

		for (unsigned j = 0; j < b.size(); ++j) { bytes[j] ^= 1; assert(!noctua_crsf::valid(bytes, b.size())); bytes[j] ^= 1; }
	}

	test_now += 3000000;

	for (unsigned i = 0; i < 5; ++i) { tick(); }

	assert(telemetry_received == 5);
	test_battery_status.timestamp = test_now; test_battery_status.remaining = NAN;
	tick(); assert(telemetry_received == 5);
	test_last.rc_failsafe = true;

	for (unsigned i = 0; i < 5; ++i) { tick(); }

	assert(telemetry_received == 5);
	noctua_crsf::Frame command(0x32); command.put(1, 1); command.finish();
	assert(!noctua_crsf::valid(command.bytes, command.size));
	noctua_crsf::Frame text(0x21);

	for (unsigned i = 0; i < 17; ++i) { text.put('x', 1); } text.put(0, 1); text.finish();

	assert(!noctua_crsf::valid(text.bytes, text.size));

	test_last.rc_failsafe = false;
	auto airspeed_cycle = [&](float value, int8_t source, int64_t age, bool expected, uint16_t wire = 0) {
		const unsigned before = telemetry_received;
		test_airspeed_validated = {uint64_t(int64_t(test_now) - age), value, source};
		for (unsigned i = 0; i < 5; ++i) { tick(); }
		assert(telemetry_received == before + unsigned(expected));
		if (expected) {
			assert(telemetry_packet.buffer[6] == 0x0a);
			assert((uint16_t(telemetry_packet.buffer[7]) << 8 | telemetry_packet.buffer[8]) == wire);
		}
	};
	airspeed_cycle(15.f, 1, 0, true, 540);
	airspeed_cycle(0.f, 2, 0, true, 0);
	airspeed_cycle(2000.f, 3, 0, true, 65535);
	airspeed_cycle(NAN, 1, 0, false);
	airspeed_cycle(INFINITY, 1, 0, false);
	airspeed_cycle(-1.f, 1, 0, false);
	for (int8_t source : {-1, 0, 4}) { airspeed_cycle(15.f, source, 0, false); }
	airspeed_cycle(15.f, 1, 1000001, false);
	airspeed_cycle(15.f, 1, -1000000, false);
	noctua_crsf::Frame oversized_airspeed(0x0a);
	oversized_airspeed.put(540, 3); oversized_airspeed.finish();
	assert(!noctua_crsf::valid(oversized_airspeed.bytes, oversized_airspeed.size));
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
static void test_redundancy()
{
	test_node_id = 42; test_backup_id = 43;
	Node receiver(1);
	Sender primary(42), backup(43), stranger(44);
	auto a = channels(), b = channels(); b.rcin[0] = 1800;
	{
		UavcanRcInputBridge bridge(receiver); assert(bridge.init() >= 0);
		const auto count = test_publications;
		backup.send(receiver, b); bridge.update(); assert(test_publications == count);
		primary.send(receiver, a); assert(test_last.values[0] == 1000 && !test_last.rc_lost);
		test_now += 80000; backup.send(receiver, b);
		assert(test_last.values[0] == 1000);
		test_now += 19999; bridge.update(); assert(test_last.values[0] == 1000);
		++test_now; bridge.update();
		assert(test_last.values[0] == 1800 && !test_last.rc_lost);
		assert(test_last.timestamp_last_signal == test_now - 20000);
		primary.send(receiver, a); assert(test_last.values[0] == 1800); // no preemption
		auto wrong = b; wrong.id = 1; wrong.rcin[0] = 900;
		backup.send(receiver, wrong); stranger.send(receiver, wrong);
		assert(test_last.values[0] == 1800);
		auto failed = b; failed.status |= Rc::STATUS_FAILSAFE;
		test_now += 10000; backup.send(receiver, failed);
		assert(!test_last.rc_lost && test_last.values[0] == 1000); // immediate failover
		auto bad = a; bad.status |= Rc::STATUS_FAILSAFE;
		primary.send(receiver, bad); assert(test_last.rc_lost && test_last.rc_failsafe);
		backup.send(receiver, b, 100000); backup.send(receiver, b, -1);
		assert(test_last.rc_lost);
		test_now += 20000; backup.send(receiver, b);
		assert(!test_last.rc_lost && test_last.values[0] == 1800);
		backup.send(receiver, failed, 10000); // older failure must not overwrite fresh data
		assert(!test_last.rc_lost);
		test_now += 100000; bridge.update(); assert(test_last.rc_lost);
		const auto lost = test_publications; bridge.update(); assert(test_publications == lost);
		primary.send(receiver, a); assert(!test_last.rc_lost && test_last.values[0] == 1000);
		backup.send(receiver, b); primary.send(receiver, channels(3));
		assert(!test_last.rc_lost && test_last.values[0] == 1800);
	}
	{
		UavcanRcInputBridge bridge(receiver); assert(bridge.init() >= 0);
		test_now += 100000; backup.send(receiver, b); // primary absent on startup
		assert(!test_last.rc_lost && test_last.values[0] == 1800);
		primary.send(receiver, a); assert(test_last.values[0] == 1800);
	}
	for (int primary_id : {0, 43, -1, 128}) {
		test_node_id = primary_id; UavcanRcInputBridge bridge(receiver); assert(bridge.init() < 0);
	}
	test_node_id = 42;
	for (int backup_id : {-1, 128}) {
		test_backup_id = backup_id; UavcanRcInputBridge bridge(receiver); assert(bridge.init() < 0);
	}
	test_backup_id = 0;
	puts("PASS: redundant receivers, startup, timeout/failsafe failover, sticky selection, source isolation, stale data, both lost and recovery");
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
	test_redundancy();
	test_telemetry();
	puts("PASS: real DroneCAN transfers, channel bounds, source locking, quality, failsafe, stale/partial transfers and timeout recovery");
}
