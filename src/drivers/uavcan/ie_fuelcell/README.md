# IE-SOAR S800 CAN telemetry

Receive-only support for the IE-SOAR S800 customer CAN **format 2 (single
system)**, based on User Manual V1.2, section 9.2.2 (printed page 34).
The fuel cell transmits an eight-byte **extended** frame with ID `0x400`
every 100 ms. The default fuel-cell bitrate in the manual is 500 kbit/s.

## Implementation

The bridge uses libuavcan's raw receive listener inside the existing PX4
`uavcan` module. It does not open another hardware driver or consume frames
from a competing receive queue. DroneCAN processing continues normally.
Only the configured interface is accepted; loopback, standard-ID, RTR, error
and wrong-length frames are ignored. The Pixhawk 6C CAN driver already accepts
all frames, so no hardware filter changes are needed.

The build option is `CONFIG_UAVCAN_IE_FUELCELL=y`, enabled in
`px4_fmu-v6c_default`. The bridge remains disabled at runtime until
`IEFC_CAN_EN=1`. It has no separate start command; it starts with `uavcan`.

The raw listener slot is exclusive. If another component owns it, this bridge
reports an error and does not replace it. Other CAN protocols must use the
same bitrate on a shared physical bus. This PX4 UAVCAN driver applies
`UAVCAN_BITRATE` to both interfaces; the bridge does not provide an independent
bitrate for CAN2.

## Configuration and bench check

1. In the fuel-cell configuration, select customer CAN **format 2**, and match
   its bitrate to `UAVCAN_BITRATE`. Format 3 uses the same ID but different
   meanings/scales and **must not** be used. It cannot be autodetected from
   this frame. Only one single-system FCPM may transmit ID `0x400` per bus.
2. On the S800 customer connector, CAN_H is pin 7, CAN_L pin 8, and ground
   pins 13/14. Pins 9/10 enable the internal 120-ohm termination when linked;
   enable it only when the FCPM is a bus endpoint. See manual section 3.2.1.
3. Enable the bridge, choose the interface and reboot. Preserve an existing
   nonzero `UAVCAN_ENABLE` setting, especially when using DroneCAN actuators.
   If UAVCAN is currently disabled, set it to 1 for telemetry-only testing.

```sh
param set IEFC_CAN_EN 1
param set IEFC_CAN_IFACE 1
param set IEFC_CAN_TOUT 500
# Set UAVCAN_BITRATE to the agreed bus bitrate (500000 for S800 factory baud).
# Set UAVCAN_ENABLE to 1 only if currently 0.
reboot
```

```sh
uavcan status
listener ie_fuelcell_can_status 10
```

`IEFC_CAN_IFACE=1` selects CAN1 and `=2` selects CAN2 on Pixhawk 6C. Parameters
are read once during initialization and require a reboot to change.

Expect increasing received-frame counts and approximately 10 Hz updates. Compare pressure,
voltage, power, state and error values against the IE diagnostic tool. Then
disconnect the telemetry cable: `connected` must become false within the
configured timeout plus scheduling delay. Reconnect and confirm recovery.
Also verify normal DroneCAN sensors/actuators on the intended bus setup.

## Output and validity

The `ie_fuelcell_can_status` topic is included in default ULog logging at up to
10 Hz. Its units are explicit:

| Measurement | Conversion |
| --- | --- |
| Tank gauge pressure | raw 10-bit value × 0.5 bar |
| Hybrid battery voltage | raw 10-bit value × 0.1 V |
| Combined output power | raw 10-bit value × 10 W |
| SM input power | raw 10-bit value × 5 W |
| Hybrid battery power | raw 10-bit value × 10 − 5000 W |

Counter, state and error are raw 4-, 4- and 6-bit values respectively. No UART
fault-subcode mapping is assumed. In particular, the battery power offset is
subtracted, matching the manual's stated range of −5000 to 5230 W.

The existing UART `fuel_cell` topic is unchanged. CAN pressure is in bar while
UART formats 4–6 report hydrogen percent; CAN also lacks regulated pressure
and UART subcodes. A separate topic prevents silently changing units for
existing consumers. There is no new MAVLink or DDS mapping in this version;
inspect through the NSH listener or ULog.

Every matching eight-byte extended frame updates measurements and reception
freshness, even if its cyclic counter is unchanged. Hardware observations show
an apparently fixed counter; the manual does not guarantee that it advances
autonomously. `duplicate_frames` counts repeated counters for diagnostics;
it does not imply that the payload is identical. Counter jumps and wraparound
are also allowed. At
startup and after timeout, `connected=false` and all floating-point
measurements are NaN. `timestamp_sample` is the last accepted reception time,
or zero before the first frame. Raw state/error fields retain their last
values and must only be used when `connected=true`. A timestamp is published
once on timeout; downstream consumers must also monitor publication age if
the entire UAVCAN module stops. `connected` indicates recent matching CAN
traffic, not proof that the decoded measurements or FCPM state are correct.

`raw_data` retains the last eight payload bytes in wire order. `uavcan status`
also prints these as hexadecimal bytes. Capture several samples alongside
the fuel-cell's own diagnostics to verify the bit layout. A powered FCPM can
transmit telemetry while its stack is off or faulted; receive-only telemetry
does not require hydrogen consumption or stack power generation.

## Scope

This bridge does not send `0x200` control frames or override fuel-cell voltage
or power limits. The manual's 500 ms cyclic-counter requirement concerns
**commands sent to the FCPM**, not a heartbeat required by this receive-only
implementation. Existing fuel-cell configuration remains in effect.

No flight failsafe or battery-state estimation is triggered by this topic.
Hardware validation of the manufacturer's CAN encoding remains necessary.
Parallel-system format 3 and dynamic voltage/power-limit control are future
work; format 3 must have its own decoder and explicit selection.

## Validation

The host test compiles the actual bridge with the repository's pinned
libuavcan core, injects frames through its real dispatcher, and stubs only PX4
parameters, time and uORB publication. It checks a fixed reference frame,
every payload bit, all 1024 battery-power encodings, malformed frames,
interface selection, loopback rejection, counter wrap/jumps/duplicates,
timeout, recovery, disabled/invalid configuration and listener ownership.
It also checks that this bridge sends no CAN frames. AddressSanitizer and
UndefinedBehaviorSanitizer are enabled.

```sh
git submodule update --init --recursive src/drivers/uavcan/libuavcan
python3 -m pip install 'setuptools<81' pyserial pyyaml
python3 src/drivers/uavcan/ie_fuelcell/test/run.py
```

In containers where LeakSanitizer cannot inspect `/proc`, use
`ASAN_OPTIONS=detect_leaks=0`; address and undefined-behavior checks remain on.

Firmware build:

```sh
git submodule update --init --recursive
make px4_fmu-v6c_default
```
