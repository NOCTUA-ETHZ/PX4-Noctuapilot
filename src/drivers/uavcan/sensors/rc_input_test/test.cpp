// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#include "rc_input.hpp"
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
	puts("PASS: real DroneCAN transfers, channel bounds, source locking, quality, failsafe, stale/partial transfers and timeout recovery");
}
