# DroneCAN RC input (Noctua PX4 v1.17)

This branch accepts `dronecan.sensors.rc.RCInput` from an AP_Periph RC adapter
and publishes it to PX4's primary `input_rc` topic. The normal RC calibration,
channel mapping, switches and manual-control loss handling remain in use.
It is disabled by default. This is an RC input bridge, not a CRSF serial tunnel:
flight-controller telemetry requires the optional [Noctua telemetry extension](rc_telemetry.md) and custom Matek firmware.

## Matek CAN-L431 and ExpressLRS

Use the stock **MatekL431-RC** AP_Periph firmware (not the generic sensor build).
No custom Matek source change is required. A node already running this RC build
only needs its configuration checked; a generic CAN-L431 needs the RC firmware
flashed using Matek's instructions.

Wire the RadioMaster RP1, or later DBR4 in CRSF serial mode, to the adapter:

| Receiver | CAN-L431 |
| --- | --- |
| TX | RX3 |
| RX | TX3 |
| GND | GND |
| Power | Regulated 5 V |

Connect CAN H, CAN L and ground to the Pixhawk CAN bus. Power the CAN-L431 from
regulated 5 V. Use a 120-ohm termination at each physical end of the bus; account
for existing terminators before adding any. Do not connect a battery directly
to the adapter's 5 V input.

Configure the **adapter's** parameters with a DroneCAN configuration tool:

- Set a fixed, unused `CAN_NODE` ID, for example `42` (distinct from the Pixhawk).
- Use the same CAN bitrate as PX4, normally 1 Mbit/s.
- Leave the RC port on its UART3 default (serial port index 2).
- Set `RC1_PORT_OPTIONS=0` for normal, non-inverted CRSF UART wiring.
- Set `RC_PROTOCOLS=512` for CRSF only, or `1` for protocol autodetection.
- Set `RC_MSGRATE=50` Hz. Rates at or below 10 Hz will trip PX4's 100 ms timeout;
  use margin above this threshold, not exactly 10 Hz.
- Reboot the adapter after configuration. Verify it broadcasts
  `dronecan.sensors.rc.RCInput`, with changing channels and the expected node ID.

These are AP_Periph parameters, not PX4 parameters. Names/availability can vary
with AP_Periph version. The stock RC target selects UART3; don't configure that
port as a generic serial peripheral. See the
[Matek RC adapter instructions](https://www.mateksys.com/?portfolio=can-l4-rc)
and [MatekL431-RC firmware](https://firmware.ardupilot.org/AP_Periph/stable/MatekL431-RC/).

## PX4 setup

Build this branch for the Pixhawk 6C using the repository's Noctua MAVLink setup:

```sh
make px4_fmu-v6c_default
```

The board already includes CRSF and DroneCAN. `CONFIG_UAVCAN_SENSOR_RC_INPUT=y`
is the new build option, enabled by default when DroneCAN is included. The
UART CRSF driver is not used to decode CAN messages.

| PX4 parameter | Setting |
| --- | --- |
| `UAVCAN_ENABLE` | `2` for sensors/automatic allocation, or retain `3` if using CAN actuators |
| `UAVCAN_BITRATE` | Match the bus, normally `1000000` |
| `UAVCAN_SUB_RC` | `1` |
| `UAVCAN_RC_ID` | Adapter's fixed node ID, for example `42` |
| `RC_CRSF_PRT_CFG` | `0` to disable the previous direct UART CRSF input |

Reboot PX4. Disconnect any other receiver from RCIN and disable other serial RC
inputs. PX4's `rc_update` reads the primary `input_rc` instance: this bridge does
not arbitrate between DroneCAN and simultaneous UART/RCIN receivers.
Keep `COM_RC_IN_MODE` configured to accept RC input. Recheck RC calibration,
channel mapping, flight-mode switches, `COM_RC_LOSS_T` and the RC-loss action.

`UAVCAN_RC_ID=0` can be used for initial setup: it locks to the first healthy
node/device pair. Even with a configured node, the first healthy device ID on
that node is locked. Selection remains locked until the UAVCAN driver restarts;
another source cannot take over on signal loss. Prefer an explicit node ID for
flight. If a node ID changes, update PX4 and reboot.

## Bench checks before use

Remove the propeller and keep actuators safe during these checks.

```sh
uavcan status
listener input_rc
listener rc_channels
listener manual_control_input
```

1. With transmitter and receiver on, verify `input_source=16`, expected channel
   count (normally 16 for CRSF), correct values, `rc_lost=false` and
   `rc_failsafe=false`. Verify every control and switch in QGroundControl.
2. Turn the transmitter off. Check that an explicit adapter failsafe or stopped
   channel stream causes RC loss. Verify the configured vehicle failsafe action.
3. Restore the transmitter and verify recovery. Then separately unplug receiver
   power, the receiver UART, and the adapter CAN connection. Each must cause
   loss. Restore each connection and confirm recovery from the selected node.
4. If multiple RC nodes exist, verify the other node cannot control the aircraft,
   including while the selected receiver is unavailable.

The bridge marks the input lost after 100 ms without a fresh healthy transfer
(plus normal driver scheduling delay). Explicit failsafe and fewer than four
channels mark loss immediately once a source is selected. Loss reports never
advance `timestamp_last_signal`, so PX4's configured manual-control timeout
continues to operate. The final vehicle response follows `COM_RC_LOSS_T` and
normal PX4 failsafe policy, not just the bridge's 100 ms interval.

Up to 18 channels are copied in microseconds; surplus channels are discarded.
Unknown quality is reported as unknown, not zero. Legacy quality is scaled from
0..255 to 0..100 RSSI; compatible extended LQ and RSSI-dBm encodings are supported.
Receiving a held stream without a failsafe flag cannot reveal radio loss, so
verify the actual receiver/adapter failsafe behavior in the bench checks above.

## Host regression test

This compiles the real bridge with the pinned libuavcan stack and generated DSDL,
then sends real serialized transfers through its dispatcher. Only PX4 platform
services are stubbed. It uses AddressSanitizer and UndefinedBehaviorSanitizer.

```sh
git submodule update --init --recursive src/drivers/uavcan/libdronecan
# In a Python virtual environment:
pip install 'setuptools<81' empy==3.3.4 pyyaml
python src/drivers/uavcan/sensors/rc_input_test/run.py
```

Tests cover node/device selection, channel bounds, quality, explicit failsafe,
short channel arrays, stale/future/partial transfers, timeout boundaries and
recovery. Host tests and a firmware build do not replace the hardware checks.
