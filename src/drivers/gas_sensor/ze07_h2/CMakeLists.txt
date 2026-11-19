# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
px4_add_module(
	MODULE drivers__gas_sensor__ze07_h2
	MAIN ze07_h2
	SRCS ZE07H2.cpp
	MODULE_CONFIG module.yaml
	DEPENDS px4_work_queue
)
