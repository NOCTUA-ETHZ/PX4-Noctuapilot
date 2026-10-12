# Fuel cell

One `fuel_cell` command and one public `fuel_cell` topic for Intelligent
Energy UART formats 4-6 or S800 customer CAN format 2. Select one input with
`FC_INTERFACE` and reboot:

| FC_INTERFACE | Input |
| --- | --- |
| 0 | Disabled (default) |
| 1 | UART |
| 2 | CAN1 |
| 3 | CAN2 |

## Setup

For UART, set `FC_SER_CFG` to the connected serial port in QGroundControl,
set `FC_BAUD` to the fuel-cell baud (default 9600), select `FC_INTERFACE=1`,
and reboot. The generated serial startup script passes the selected device
to the driver and participates in PX4's serial-port allocation. Manual bench
startup uses `fuel_cell start -d /dev/ttyS1` with the actual device path.
The parser validates the documented ASCII checksum and accepts CR, LF and
CRLF endings. Invalid records do not refresh telemetry. Formats 1-3 are not
supported by this UART parser.

For CAN1:

```sh
param set FC_INTERFACE 2
param set FC_SER_CFG 0
param set FC_TIMEOUT 2000
# UAVCAN_ENABLE must be nonzero; preserve an existing actuator configuration.
# UAVCAN_BITRATE must match the fuel cell (normally 500000).
reboot
```

Set FC_SER_CFG to Disabled when using CAN to release its serial-port reservation.
Select customer CAN format **2** on the fuel cell. Format 3 has different
scales and cannot be distinguished by CAN ID. CAN2 is `FC_INTERFACE=3`.
The existing UAVCAN module owns both CAN interfaces; no second CAN hardware
driver is opened. UAVCAN_BITRATE applies to both buses. Only one fuel cell
may transmit extended ID 0x400 on the selected bus. The S800 customer
connector uses CAN_H pin 7, CAN_L pin 8, ground pins 13/14; linking pins 9/10
enables its 120-ohm termination (only at a bus endpoint).

The CAN backend starts with UAVCAN and the common driver starts afterward.
The common driver is the only local driver publishing the public topic.
`fuel_cell stop` publishes disconnected/NaN once and stops that publisher;
the UAVCAN backend can keep receiving internal diagnostics. A manual
`fuel_cell start` resumes publication. Change interfaces by rebooting, as
both backend and front end read parameters at startup. `FC_TIMEOUT` applies
to both inputs and defaults to 2000 ms to accommodate UART update periods.

```sh
fuel_cell status
listener fuel_cell
fuel_cell stop
```

## Message compatibility

All existing `FuelCell.msg` field names and UART meanings are retained.
`tankpressure` is the legacy name for UART **hydrogen percentage**, not bar.
The following additions are necessary to represent both inputs honestly:

- `tank_pressure_bar`: CAN tank gauge pressure; NaN on UART.
- `timestamp_sample`: last accepted reception time, distinct from publication.
- `source`: selected interface, using the FC_INTERFACE values above.
- `connected`: fresh telemetry; it does not mean the stack is running.
- `raw_can_error`: CAN error bits; -1 on UART or when stale.

On CAN, `tankpressure` and `regpressure` are NaN. `mainerror`/`suberror` remain
UART fields and are -1 on CAN; raw CAN faults are not assumed equivalent.
On startup, stop or timeout, floats become NaN and integer state/errors -1.
`timestamp_sample` is retained on timeout. The driver also detects a stopped
UAVCAN backend by checking sample age itself. Consumers must check freshness
if the entire driver stops unexpectedly.

DDS publishes `/fmu/out/fuel_cell`. **The companion needs this custom
message schema**. Copy the updated `msg/FuelCell.msg` into the companion's px4_msgs
package and rebuild affected ROS 2 packages before using DDS with this
firmware. No automatic changes are made to a companion or its message repo.

## CAN validation (MCU v3.63, customer format 2)

Paired raw CAN and manufacturer telemetry from one run confirm the output
power (10 W resolution), SPM output power (5 W) and battery power (10 W).
These are now published as `outputpower`, `spmpower` and `battpower`.
The internal `stack_power_w` name is retained for compatibility, but its
measurement matches the manufacturer's **SPM_Output_Power**, not Stack_Power.
Positive battery power means charging; negative means discharging.

Battery power uses `(byte6 * 8 + (byte7 >> 5)) * 10 - 5000`.
The previous decoder dropped byte 7 bit 5 and included it in the error code,
causing approximately -2500 W at idle and spurious error 32 transitions.
For example, `80 00 1c e0 00 01 3e 60` now decodes to 46.2 V, 0 W output,
5 W SPM output, -10 W battery and low-five-bit raw error 0.

Validation covered 5,685 frames, including charging and discharging.
The recording clocks required approximately 402.4 seconds of alignment;
values are not expected to match exactly across the 10 Hz CAN and 1 Hz CSV.
The observed battery range was -2300 to +220 W. The upper encoding range and
nonzero low-five-bit fault meanings were not exercised by this recording.
A firmware-specific protocol specification or paired fault capture is still
required to establish fault meanings; no UART main/suberror is synthesized.
UART decoding and the public message schema are unchanged.

There are no CAN control transmissions, voltage/power overrides or new
flight failsafes. This is receive-only telemetry.

## Migration and validation

`fuel_cell` replaces the old `ie_fuelcell` command. `FC_INTERFACE`,
`FC_SER_CFG`, `FC_BAUD` and `FC_TIMEOUT` replace the IEFC_* configuration;
old saved parameters do not enable this driver. The previous
`ie_fuelcell_can_status` topic is replaced by internal `fuel_cell_can`.
Normal consumers should use `fuel_cell` for either transport.

Build options are `CONFIG_DRIVERS_FUEL_CELL=y` and (for the CAN backend)
`CONFIG_UAVCAN_FUEL_CELL=y`; both are enabled for Pixhawk 6C in this branch.

```sh
ASAN_OPTIONS=detect_leaks=0 python3 src/drivers/fuel_cell/test/run.py
ASAN_OPTIONS=detect_leaks=0 python3 src/drivers/uavcan/fuel_cell/test/run.py
make px4_fmu-v6c_default
```

The first test covers UART checksum/framing, malformed/empty fields,
resynchronization, shared-topic mapping, unavailable values and freshness.
The second compiles the real CAN bridge with the pinned libuavcan dispatcher
and covers wire decoding, filters, counter behavior, timeouts and lifecycle.
After flashing, validate UART and CAN separately, disconnect/reconnect each
input, and verify stop/restart and boot startup on the configured port.

## PX4 1.17 port

Consolidated from feature/ie_fc_can_s800 at 70c000ef (including a1a5cbff,
3886603 and 1e0cd744). This target did not have the original UART driver.
Integration preserves the target board settings and uses its in-tree
libdronecan library. DDS output is enabled; no DDS input publisher is added.
