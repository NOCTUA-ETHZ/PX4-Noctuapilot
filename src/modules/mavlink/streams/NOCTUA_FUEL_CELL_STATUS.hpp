// Copyright (c) 2026 Noctua. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <uORB/topics/fuel_cell.h>

class MavlinkStreamNoctuaFuelCellStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamNoctuaFuelCellStatus(mavlink); }

	static constexpr const char *get_name_static() { return "NOCTUA_FUEL_CELL_STATUS"; }

	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NOCTUA_FUEL_CELL_STATUS; }

	const char *get_name() const override { return get_name_static(); }

	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _subscription.advertised() ? MAVLINK_MSG_ID_NOCTUA_FUEL_CELL_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamNoctuaFuelCellStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _subscription{ORB_ID(fuel_cell)};
	bool send() override
	{
		fuel_cell_s sample{};

		if (!_subscription.update(&sample)) { return false; }

		mavlink_noctua_fuel_cell_status_t msg{};
		msg.time_usec = sample.timestamp;
		msg.sample_time_usec = sample.timestamp_sample;
		msg.hydrogen_remaining_pct = sample.tankpressure;
		msg.regulated_pressure_bar = sample.regpressure;
		msg.tank_pressure_bar = sample.tank_pressure_bar;
		msg.voltage = sample.voltage;
		msg.output_power = sample.outputpower;
		msg.stack_power = sample.spmpower;
		msg.battery_power = sample.battpower;
		msg.state = sample.psustate;
		msg.main_error = sample.mainerror;
		msg.sub_error = sample.suberror;
		msg.raw_can_error = sample.raw_can_error;
		msg.source = sample.source;
		msg.connected = sample.connected;
		mavlink_msg_noctua_fuel_cell_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};
