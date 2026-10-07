// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "Protocol.hpp"

#include <uavcan/node/abstract_node.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/fuel_cell_can.h>

class FuelCellCanBridge : public uavcan::IRxFrameListener
{
public:
	explicit FuelCellCanBridge(uavcan::INode &node) : _node(node) {}
	~FuelCellCanBridge() override;

	// All methods run under the owning UavcanNode mutex, in its work queue,
	// except init/destruction, when the node is not spinning.
	void init();
	void update();
	void print_status() const;
	void handleRxFrame(const uavcan::CanRxFrame &frame, uavcan::CanIOFlags flags) override;

private:
	void invalidate(uint64_t now);

	uavcan::INode &_node;
	uORB::Publication<fuel_cell_can_s> _status_pub{ORB_ID(fuel_cell_can)};
	fuel_cell_can_s _status{};
	fuelcell_can::Freshness _freshness;
	uint64_t _timeout_us{500000};
	uint32_t _invalid_frames{0};
	bool _enabled{false};
};
