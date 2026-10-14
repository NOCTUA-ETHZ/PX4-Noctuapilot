# Noctua ground-control integration

`feature/noctua-gc-integration` targets `flash_v1.17`. Host simulation backports,
the local Protobuf compatibility adjustment, MAVLink integration and the
opt-in SITL telemetry demo are separate commits for review.

## Shared MAVLink definitions

Clone the private `NOCTUA-ETHZ/noctua-mavlink` repository beside this checkout,
initialize its `pymavlink` submodule and use the reviewed revision
`e81cd04f48f2b67c0c79ba5df83a3b8a66f613dc`, matching the noctua-GC preview.
Repository access is required; the XML definitions are not copied into PX4.

Set `NOCTUA_MAVLINK_SOURCE_DIR` when configuring to build the Noctua dialect.
With that option unset, the existing PX4 MAVLink submodule/dialect is used and
the Noctua streams are excluded. Normal/GCS mode sends fuel-cell status and
gas-probe samples at up to 2 Hz and CAN diagnostics at up to 1 Hz, when their
uORB topics publish. UART and CAN retain their own units and availability;
the newer base branch supplies validated MCU v3.63 CAN power decoding.

## Pixhawk 6C firmware on macOS

The board target is `px4_fmu-v6c_default`. It already includes the UART/CAN
fuel-cell and DFRobot gas-sensor drivers. Run from this repository root after
the normal PX4 macOS development setup; Arm GNU GCC and Ninja must be on PATH.
The Gazebo environment is needed only for simulation.

```sh
git switch feature/noctua-gc-integration
source .venv/bin/activate
export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer

cmake -S . -B build/px4_fmu-v6c_noctua -G Ninja \
  -DCONFIG=px4_fmu-v6c_default \
  -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python" \
  -DNOCTUA_MAVLINK_SOURCE_DIR="$PWD/../noctua-mavlink" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build/px4_fmu-v6c_noctua --parallel 4
```

The firmware is `build/px4_fmu-v6c_noctua/px4_fmu-v6c_default.px4`.
In noctua-GC/QGC, open **Vehicle Setup → Firmware**, enable **Advanced settings**,
choose **Custom firmware file**, and select that `.px4` file. Follow the USB
reconnection prompts. The SITL-only telemetry demo is not included in this
hardware configuration; fuel-cell/probe values require the real drivers and
their transport parameters to be configured.

## Simulation

Use [the locked macOS Gazebo setup](Tools/setup/macos/README.md) and the
[Noctua telemetry demo instructions](src/modules/simulation/noctua_telemetry_sim/README.md).
`px4_sitl_noctua` includes the demo module, but it publishes only after an
explicit start or `PX4_NOCTUA_TELEMETRY=1`. It sends synthetic display samples,
does not model fuel-cell physics and does not change flight control/failsafes.

Verify its real uORB-to-MAVLink transport using the isolated test:

```sh
python src/modules/simulation/noctua_telemetry_sim/test/smoke.py \
  --build-dir build/px4_sitl_noctua --mavlink-dir ../noctua-mavlink
```

Upstream provenance and local simulation compatibility changes are documented
in [UPSTREAM_BACKPORTS.md](Tools/setup/macos/UPSTREAM_BACKPORTS.md).
