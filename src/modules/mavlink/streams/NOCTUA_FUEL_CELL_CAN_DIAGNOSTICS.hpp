// Copyright (c) 2026 Noctua. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cstring>
#include <uORB/topics/fuel_cell_can.h>

class MavlinkStreamNoctuaFuelCellCanDiagnostics : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamNoctuaFuelCellCanDiagnostics(mavlink); }

	static constexpr const char *get_name_static() { return "NOCTUA_FUEL_CELL_CAN_DIAGNOSTICS"; }

	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NOCTUA_FUEL_CELL_CAN_DIAGNOSTICS; }

	const char *get_name() const override { return get_name_static(); }

	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _subscription.advertised() ? MAVLINK_MSG_ID_NOCTUA_FUEL_CELL_CAN_DIAGNOSTICS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamNoctuaFuelCellCanDiagnostics(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _subscription{ORB_ID(fuel_cell_can)};
	bool send() override
	{
		fuel_cell_can_s sample{};

		if (!_subscription.update(&sample)) { return false; }

		mavlink_noctua_fuel_cell_can_diagnostics_t msg{};
		msg.time_usec = sample.timestamp;
		msg.sample_time_usec = sample.timestamp_sample;
		msg.tank_pressure_bar = sample.tank_pressure_bar;
		msg.battery_voltage = sample.battery_voltage_v;
		msg.output_power = sample.output_power_w;
		msg.stack_power = sample.stack_power_w;
		msg.battery_power = sample.battery_power_w;
		msg.received_frames = sample.received_frames;
		msg.duplicate_frames = sample.duplicate_frames;
		msg.can_interface = sample.interface;
		msg.counter = sample.counter;
		msg.state = sample.state;
		msg.error_code = sample.error_code;
		msg.connected = sample.connected;
		memcpy(msg.raw_data, sample.raw_data, sizeof(msg.raw_data));
		mavlink_msg_noctua_fuel_cell_can_diagnostics_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};
