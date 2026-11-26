/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "rc_input.hpp"
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <math.h>

UavcanRcInputBridge::UavcanRcInputBridge(uavcan::INode &node) : _node(node), _sub_rc(node)
{
	_report.input_source = input_rc_s::RC_INPUT_SOURCE_DRONECAN;
	_report.rc_lost = true;
	clear_quality(_report);
}

int UavcanRcInputBridge::init()
{
	param_get(param_find("UAVCAN_RC_ID"), &_configured_node);
	param_get(param_find("UAVCAN_RC_BK_ID"), &_backup_node);
	_start_time = hrt_absolute_time();

	if (_configured_node < 0 || _configured_node > 127 || _backup_node < 0 || _backup_node > 127
	    || (_backup_node != 0 && (_configured_node == 0 || _backup_node == _configured_node))) {
		return -1;
	}

	return _sub_rc.start(Callback(this, &UavcanRcInputBridge::rc_sub_cb));
}

void UavcanRcInputBridge::clear_quality(input_rc_s &report)
{
	report.rssi = -1;
	report.link_quality = -1;
	report.rssi_dbm = NAN;
}

void UavcanRcInputBridge::rc_sub_cb(const uavcan::ReceivedDataStructure<RcInput> &msg)
{
	const int source = msg.getSrcNodeID().get();

	const int slot = _backup_node != 0 && source == _backup_node ? 1 : 0;
	Receiver &receiver = _receivers[slot];

	if (source == 0 || (slot == 0 && _configured_node != 0 && source != _configured_node)
	    || (receiver.node >= 0 && (source != receiver.node || msg.id != receiver.device))) {
		return;
	}

	// The libuavcan clock and HRT need not have the same epoch. Preserve the
	// receive age so queued/stale transfers cannot refresh the RC loss timer.
	const uint64_t now = hrt_absolute_time();
	const int64_t age = (_node.getMonotonicTime() - msg.getMonotonicTimestamp()).toUSec();

	if (age < 0 || age >= static_cast<int64_t>(SIGNAL_TIMEOUT_US) || static_cast<uint64_t>(age) > now) {
		return;
	}

	const uint64_t sample_time = now - age;
	const bool failsafe = msg.status & RcInput::STATUS_FAILSAFE;
	const bool healthy = !failsafe && msg.rcin.size() >= 4;

	// Never let delayed transfers replace newer receiver state.
	if (receiver.last_sample != 0 && sample_time < receiver.last_sample) {
		return;
	}

	if (receiver.node < 0) {
		if (!healthy) { return; }

		receiver.node = source;
		receiver.device = msg.id;
	}

	receiver.last_sample = sample_time;
	input_rc_s &report = receiver.report;
	report.input_source = input_rc_s::RC_INPUT_SOURCE_DRONECAN;

	report.timestamp = now;
	report.rc_total_frame_count++;
	report.rc_failsafe = failsafe;
	report.rc_lost = !healthy;
	report.channel_count = msg.rcin.size() < input_rc_s::RC_INPUT_MAX_CHANNELS ?
				msg.rcin.size() : input_rc_s::RC_INPUT_MAX_CHANNELS;

	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; ++i) {
		report.values[i] = i < report.channel_count ? msg.rcin[i] : 0;
	}

	clear_quality(report);

	if (msg.status & RcInput::STATUS_QUALITY_VALID) {
		// Extended quality types are wire compatible with the pinned DSDL.
		// Type zero is the original 0..255 RSSI; never interpret other types as RSSI.
		switch (msg.status & 0x1c) {
		case 0:
			report.rssi = (unsigned(msg.quality) * 100 + 127) / 255;
			break;

		case 4: // LQ percentage with antenna in the high bit
			if ((msg.quality & 0x7f) <= 100) {
				report.link_quality = msg.quality & 0x7f;
			}

			break;

		case 8: // Inverted RSSI in dBm; zero means unknown
			if (msg.quality != 0) {
				report.rssi_dbm = -float(msg.quality);
			}

			break;
		}
	}

	if (healthy) {
		report.timestamp_last_signal = sample_time;

	} else {
		report.rc_lost_frame_count++;
	}

	select_and_publish(now, slot);
}

void UavcanRcInputBridge::select_and_publish(uint64_t now, int updated)
{
	auto healthy = [&](int slot) {
		const auto &r = _receivers[slot];
		return r.node >= 0 && !r.report.rc_lost && !r.report.rc_failsafe
		       && r.report.timestamp_last_signal <= now
		       && now - r.report.timestamp_last_signal < SIGNAL_TIMEOUT_US;
	};
	const int previous = _active;

	if (_active < 0 || !healthy(_active)) {
		if (healthy(0)) {
			_active = 0;

		} else if (_backup_node != 0 && healthy(1)
			   && (previous >= 0 || now - _start_time >= SIGNAL_TIMEOUT_US)) {
			// Allow the primary one timeout interval to start before choosing backup.
			_active = 1;
		}
	}

	if (_active < 0) { return; }

	const bool changed = previous != _active;

	if (changed) {
		_selected_node = _receivers[_active].node;
		PX4_INFO("RC selected node %d (previous %d)", _selected_node,
			 previous >= 0 ? _receivers[previous].node : -1);
	}

	const bool valid = healthy(_active);

	if (!changed && updated != _active && (valid || _report.rc_lost)) { return; }

	const auto total = _report.rc_total_frame_count;
	const auto lost = _report.rc_lost_frame_count;
	_report = _receivers[_active].report;
	_report.timestamp = now;
	_report.rc_lost = !valid;
	_report.rc_total_frame_count = total + (updated == _active ? 1 : 0);
	_report.rc_lost_frame_count = lost + (!valid ? 1 : 0);

	if (!valid) { clear_quality(_report); }

	_input_rc_pub.publish(_report);
}

void UavcanRcInputBridge::update()
{
	select_and_publish(hrt_absolute_time());
}

void UavcanRcInputBridge::print_status() const
{
	PX4_INFO("RC input: primary %ld backup %ld selected %d lost %d",
		 (long)_configured_node, (long)_backup_node, _selected_node, _report.rc_lost);

	for (const auto &r : _receivers) {
		PX4_INFO("  node %d device %d last sample %llu failsafe %d", r.node, r.device,
			 (unsigned long long)r.last_sample, r.report.rc_failsafe);
	}
}
