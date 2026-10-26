// Copyright (c) 2026 Noctua contributors. SPDX-License-Identifier: BSD-3-Clause
#include "rc_telemetry.hpp"
#include "crsf_telemetry_wire.hpp"
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <matrix/math.hpp>
#include <cmath>
#include <cstring>
static uint32_t bounded(double v, uint32_t limit)
{
	return !std::isfinite(v) || v <= 0 ? 0 : v >= limit ? limit : uint32_t(round(v));
}
static bool fresh(uint64_t stamp, uint64_t now, uint64_t age = 1000000)
{
	return stamp != 0 && stamp <= now && now - stamp <= age;
}
int UavcanRcTelemetry::init()
{
	param_get(param_find("UAVCAN_RC_ID"), &_target);

	if (_target < 1 || _target > 127) {
		_target = 0;
		PX4_WARN("RC telemetry needs fixed UAVCAN_RC_ID");
		return 0; // A telemetry misconfiguration must not stop CAN/RC input.
	}

	_publisher.setPriority(uavcan::TransferPriority::Lowest);
	_publisher.setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
	return 0;
}
void UavcanRcTelemetry::update()
{
	const uint64_t now = hrt_absolute_time();

	if (now - _last < 100000 || _target == 0) { return; }

	_last = now;
	input_rc_s rc{};

	if (!_rc.copy(&rc) || rc.input_source != input_rc_s::RC_INPUT_SOURCE_DRONECAN ||
	    rc.rc_lost || rc.rc_failsafe || !fresh(rc.timestamp_last_signal, now, 100000)) { return; }

	noctua_crsf::Frame frame(0);

	switch (_next++ % 4) {
	case 0: {
			battery_status_s b{};

			if (!_battery.copy(&b) || !fresh(b.timestamp, now) || !b.connected ||
			    !std::isfinite(b.voltage_v) || !std::isfinite(b.current_a) ||
			    !std::isfinite(b.discharged_mah) || !std::isfinite(b.remaining) || b.remaining < 0) { return; }

			frame.bytes[6] = 0x08;
			frame.put(bounded(b.voltage_v * 10., 65535), 2);
			frame.put(bounded(b.current_a * 10., 65535), 2);
			frame.put(bounded(b.discharged_mah, 0xffffff), 3);
			frame.put(bounded(b.remaining * 100., 100), 1);
			break;
		}

	case 1: {
			sensor_gps_s g{};

			if (!_gps.copy(&g) || !fresh(g.timestamp, now, 2000000) || g.fix_type < 3 ||
			    !std::isfinite(g.latitude_deg) || !std::isfinite(g.longitude_deg) ||
			    !std::isfinite(g.altitude_msl_m) || !std::isfinite(g.vel_m_s) || !std::isfinite(g.cog_rad) ||
			    fabs(g.latitude_deg) > 90 || fabs(g.longitude_deg) > 180) { return; }

			frame.bytes[6] = 0x02;
			frame.put(uint32_t(int32_t(round(g.latitude_deg * 1e7))), 4);
			frame.put(uint32_t(int32_t(round(g.longitude_deg * 1e7))), 4);
			frame.put(bounded(g.vel_m_s * 36., 65535), 2); // 0.1 km/h
			const double heading = fmod(double(g.cog_rad) * 180. / M_PI + 360., 360.);
			frame.put(bounded(heading * 100., 35999), 2);
			frame.put(bounded(g.altitude_msl_m + 1000., 65535), 2);
			frame.put(g.satellites_used, 1);
			break;
		}

	case 2: {
			vehicle_attitude_s a{};

			if (!_attitude.copy(&a) || !fresh(a.timestamp, now)) { return; }

			const matrix::Eulerf e{matrix::Quatf(a.q)};

			if (!std::isfinite(e(0)) || !std::isfinite(e(1)) || !std::isfinite(e(2))) { return; }

			frame.bytes[6] = 0x1e;
			frame.put(uint16_t(int16_t(e(1) * 10000.f)), 2);
			frame.put(uint16_t(int16_t(e(0) * 10000.f)), 2);
			frame.put(uint16_t(int16_t(e(2) * 10000.f)), 2);
			break;
		}

	case 3: {
			vehicle_status_s s{};

			if (!_status.copy(&s) || !fresh(s.timestamp, now, 2000000)) { return; }

			const char *mode = "Other";

			switch (s.nav_state) {
			case vehicle_status_s::NAVIGATION_STATE_MANUAL: mode = "Manual"; break;

			case vehicle_status_s::NAVIGATION_STATE_ALTCTL: mode = "Altitude"; break;

			case vehicle_status_s::NAVIGATION_STATE_POSCTL: mode = "Position"; break;

			case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION: mode = "Mission"; break;

			case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: mode = "Return"; break;

			case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER: mode = "Hold"; break;

			case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: mode = "Offboard"; break;

			case vehicle_status_s::NAVIGATION_STATE_STAB: mode = "Stabilized"; break;

			case vehicle_status_s::NAVIGATION_STATE_ACRO: mode = "Acro"; break;
			}

			frame.bytes[6] = 0x21;

			for (unsigned i = 0; i <= strlen(mode); ++i) { frame.put(mode[i], 1); }

			break;
		}
	}

	frame.finish();

	if (!noctua_crsf::valid(frame.bytes, frame.size)) { return; }

	uavcan::tunnel::Targetted msg;
	msg.protocol.protocol = uavcan::tunnel::Protocol::UNDEFINED;
	msg.target_node = _target;
	msg.serial_id = -1;
	msg.options = 0;
	msg.baudrate = 0;

	for (unsigned i = 0; i < frame.size; ++i) { msg.buffer.push_back(frame.bytes[i]); }

	if (_publisher.broadcast(msg) >= 0) { ++_sent; } else { ++_failed; }
}
void UavcanRcTelemetry::print_status() const
{
	PX4_INFO("RC telemetry: target %ld sent %u failed %u", (long)_target, _sent, _failed);
}
