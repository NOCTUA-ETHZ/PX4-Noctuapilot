// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Enable IE-SOAR S800 CAN telemetry
 *
 * Requires a build with UAVCAN_IE_FUELCELL and UAVCAN_ENABLE > 0.
 * Set the fuel cell customer CAN format to 2 (single system).
 * Uses UAVCAN_BITRATE; all devices on the bus must use that bitrate.
 * Receives extended ID 0x400 only. No fuel-cell commands are sent.
 *
 * @boolean
 * @reboot_required true
 * @group IE Fuelcell
 */
PARAM_DEFINE_INT32(IEFC_CAN_EN, 0);

/**
 * IE-SOAR CAN interface
 *
 * One-based interface index in the UAVCAN driver.
 * On Pixhawk 6C, 1 selects CAN1 and 2 selects CAN2.
 * Only one single-system FCPM may use ID 0x400 on the selected bus.
 *
 * @min 1
 * @max 2
 * @value 1 CAN1
 * @value 2 CAN2
 * @reboot_required true
 * @group IE Fuelcell
 */
PARAM_DEFINE_INT32(IEFC_CAN_IFACE, 1);

/**
 * IE-SOAR CAN telemetry timeout
 *
 * Telemetry is marked disconnected if no frame with a changed cyclic
 * counter has been accepted within this interval. Invalid frames and
 * repeated counters do not refresh the timeout. No flight action is taken.
 *
 * @unit ms
 * @min 200
 * @max 5000
 * @reboot_required true
 * @group IE Fuelcell
 */
PARAM_DEFINE_INT32(IEFC_CAN_TOUT, 500);
