# Noctua CRSF telemetry over DroneCAN

This separate feature branch adds battery, GPS, attitude and flight-mode
telemetry to the RC input bridge. It requires the **custom MatekL431-RC
firmware** supplied with this change. RP1/DBR4 keep their normal ELRS firmware
and CRSF UART mode. Stock Matek firmware does not forward these messages.

## Setup

Complete [RC input setup](rc_input.md) first. Wire receiver TX to Matek RX3,
receiver RX to Matek TX3, GND to GND and regulated 5 V. Both UART wires are needed.

Example configuration (change node IDs to match your bus):

| Device | Parameter | Value |
| --- | --- | --- |
| PX4 | UAVCAN_NODE_ID | 1 |
| PX4 | UAVCAN_SUB_RC | 1 |
| PX4 | UAVCAN_RC_ID | 42 |
| PX4 | UAVCAN_RC_TEL | 1 |
| PX4 | RC_CRSF_PRT_CFG | 0 |
| Matek | CAN_NODE | 42 |
| Matek | RC_TEL_NODE | 1 |
| Matek | RC_PROTOCOLS | 512 |
| Matek | RC1_PORT_OPTIONS | 0 |
| Matek | RC_MSGRATE | 50 |

Keep DroneCAN enabled and its bitrate matching on both nodes (normally 1 Mbit/s).
Reboot both devices after configuration. `RC_CRSF_TEL_EN` is for direct UART
CRSF and does not enable the CAN return path. `RC_TEL_NODE=0` disables Matek
forwarding. PX4 needs a fixed `UAVCAN_RC_ID`; zero disables telemetry without
stopping RC input or other CAN functionality.

On the radio, enable ELRS telemetry (`Std`, not `Off` or armed-suppressed `Race`)
and use EdgeTX's Discover new sensors. Radio refresh rate depends on the ELRS
packet rate and telemetry ratio.

## What is sent

- Primary battery voltage/current, consumed mAh and remaining percent. Unknown
  remaining capacity suppresses this frame. Negative charging current becomes
  zero because CRSF's current field is unsigned.
- GPS latitude/longitude, groundspeed, course, MSL altitude, satellite count.
  A valid 3D fix is required. Speed is encoded as 0.1 km/h, altitude with the
  CRSF +1000 m offset.
- Attitude: pitch, roll, yaw.
- Common flight modes; unmapped modes display `Other`.

The sender rotates one frame every 100 ms: up to 2.5 Hz per type before radio
bandwidth limits. Stale/invalid source data skips its slot. No fuel-cell-specific
fields, MAVLink, binding commands or configuration messages are forwarded.

## Transport and limits

A private Noctua envelope uses `uavcan.tunnel.Targetted` with protocol=255
(UNDEFINED), serial_id=-1, options=0, baudrate=0. Buffer bytes `4e 43 54 01`
precede one complete CRSF frame. This is not a standardized DroneCAN CRSF
protocol. Both sources contain the same `crsf_telemetry_wire.hpp`.

PX4 uses lowest CAN priority and a 20 ms transmit deadline. Matek checks source
and destination node, envelope, frame length, CRC and telemetry type; commands
are rejected. It rejects transfers older than 200 ms, limits forwarding to
50 Hz and drops frames if UART space is insufficient. No retry queue, explicit
flush, UART lock or baud change is introduced. Source filtering is not
cryptographic authentication.

Both sides stop forwarding on RC loss. PX4 suppresses stale uORB samples.
The radio must indicate telemetry loss; no fake zero measurements are sent.
`uavcan status` reports queued/failed transmissions, not confirmed RF delivery.

## Bench acceptance (not yet performed on hardware)

With propeller removed and actuators secured:

1. Confirm controls and switches work before enabling telemetry.
2. Run `uavcan status` and check the telemetry target and increasing sent count.
3. Discover radio sensors. Compare battery values to QGC; with a GPS fix, compare
   position, MSL altitude, course, speed and satellite count. Test mode changes.
4. Separately turn off the transmitter, unplug receiver power/UART and disconnect
   CAN. Verify RC failsafe, telemetry becoming stale and recovery on reconnection.
5. Set an incorrect `RC_TEL_NODE`: telemetry must stop while RC input continues.
6. Repeat with all normal CAN devices running, checking RC latency and stability.

## Software validation

The host test compiles the real PX4 bridge and pinned DroneCAN stack, sends
serialized transfers through the dispatcher, and validates CRSF using the same
validator compiled into Matek. It checks units, bounds, CRC, rejected commands,
source selection, loss recovery, stale data and disabled telemetry. It does not
simulate STM32 UART or RF timing.

```sh
python src/drivers/uavcan/sensors/rc_input_test/run.py
make px4_fmu-v6c_default
```

Matek build: apply the supplied patch at its recorded ArduPilot revision, then
`./waf configure --board MatekL431-RC` and `./waf AP_Periph`.
