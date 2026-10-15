// Copyright (c) 2026 Noctua. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_gas_concentration.h>

class MavlinkStreamNoctuaGasSensor : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamNoctuaGasSensor(mavlink); }

	static constexpr const char *get_name_static() { return "NOCTUA_GAS_SENSOR"; }

	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NOCTUA_GAS_SENSOR; }

	const char *get_name() const override { return get_name_static(); }

	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _subscriptions.advertised_count() * (MAVLINK_MSG_ID_NOCTUA_GAS_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamNoctuaGasSensor(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<sensor_gas_concentration_s> _subscriptions{ORB_ID::sensor_gas_concentration};
	bool send() override
	{
		bool sent = false;

		for (auto &subscription : _subscriptions) {
			sensor_gas_concentration_s sample{};

			if (!subscription.update(&sample)) { continue; }

			mavlink_noctua_gas_sensor_t msg{};
			msg.time_usec = sample.timestamp;
			msg.sample_time_usec = sample.timestamp_sample;
			msg.device_id = sample.device_id;
			msg.concentration = sample.concentration_ppm;
			msg.temperature = sample.temperature;
			msg.gas_type = sample.gas_type;
			msg.instance = subscription.get_instance();
			mavlink_msg_noctua_gas_sensor_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};
