// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "Protocol.hpp"

#include <uavcan/node/abstract_node.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ie_fuelcell_can_status.h>

class IeFuelcellCanBridge : public uavcan::IRxFrameListener
{
public:
	explicit IeFuelcellCanBridge(uavcan::INode &node) : _node(node) {}
	~IeFuelcellCanBridge() override;

	// All methods run under the owning UavcanNode mutex, in its work queue,
	// except init/destruction, when the node is not spinning.
	void init();
	void update();
	void print_status() const;
	void handleRxFrame(const uavcan::CanRxFrame &frame, uavcan::CanIOFlags flags) override;

private:
	void invalidate(uint64_t now);

	uavcan::INode &_node;
	uORB::Publication<ie_fuelcell_can_status_s> _status_pub{ORB_ID(ie_fuelcell_can_status)};
	ie_fuelcell_can_status_s _status{};
	ie_fuelcell::Freshness _freshness;
	uint64_t _timeout_us{500000};
	uint32_t _invalid_frames{0};
	bool _enabled{false};
};
