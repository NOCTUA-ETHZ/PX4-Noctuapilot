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

#pragma once

#include "sensor_bridge.hpp"
#include <dronecan/sensors/rc/RCInput.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/input_rc.h>

// RC uses the primary input_rc instance; it is not a redundant sensor topic.
class UavcanRcInputBridge : public IUavcanSensorBridge
{
public:
	explicit UavcanRcInputBridge(uavcan::INode &node);
	const char *get_name() const override { return "rc_input"; }
	int init() override;
	void update() override;
	void print_status() const override;
	unsigned get_num_redundant_channels() const override { return _selected_node >= 0 ? 1 : 0; }

private:
	using RcInput = dronecan::sensors::rc::RCInput;
	using Callback = uavcan::MethodBinder<UavcanRcInputBridge *,
	      void (UavcanRcInputBridge::*)(const uavcan::ReceivedDataStructure<RcInput> &)>;
	void rc_sub_cb(const uavcan::ReceivedDataStructure<RcInput> &msg);
	void clear_quality();
	static constexpr uint64_t SIGNAL_TIMEOUT_US = 100000;

	uavcan::INode &_node;
	uavcan::Subscriber<RcInput, Callback> _sub_rc;
	uORB::Publication<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};
	input_rc_s _report{};
	int32_t _configured_node{0};
	int _selected_node{-1};
	int _selected_device{-1};
};
