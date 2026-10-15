# Noctua ground-control integration

`feature/noctua-gc-integration` targets `flash_v1.17`. Host simulation backports,
the local Protobuf compatibility adjustment, MAVLink integration and the
opt-in SITL telemetry demo are separate commits for review.

## Shared MAVLink definitions

PX4 pins the private `NOCTUA-ETHZ/noctua-mavlink` repository at
`src/modules/mavlink/mavlink`, revision
`e81cd04f48f2b67c0c79ba5df83a3b8a66f613dc`, matching the noctua-GC preview.
The Noctua dialect is selected for all Noctua PX4 board builds automatically.
Repository access and GitHub SSH authentication are required; the private XML
is not copied into the public PX4 repository.

After pulling this branch into an existing checkout, update the dependency:

```sh
git submodule sync -- src/modules/mavlink/mavlink
git submodule update --init --recursive src/modules/mavlink/mavlink
```

Normal builds need no external MAVLink path. `NOCTUA_MAVLINK_SOURCE_DIR` remains
an optional override for developing changes in a separate checkout.
Normal/GCS and Onboard modes (including USB) send fuel-cell status and
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

# Current CMake needs this for the older bundled third-party projects.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
make px4_fmu-v6c_default
```

The firmware is `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`.
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
  --build-dir build/px4_sitl_noctua --mavlink-dir src/modules/mavlink/mavlink
```

Upstream provenance and local simulation compatibility changes are documented
in [UPSTREAM_BACKPORTS.md](Tools/setup/macos/UPSTREAM_BACKPORTS.md).

## GitHub CI

The shared `setup-noctua-mavlink` action checks out the exact gitlink revision
using `NOCTUA_MAVLINK_READ_KEY`, a read-only deploy key scoped to the private
MAVLink repository. Credentials are removed after checkout. Builds that require
that dependency must run on trusted branches with access to this secret;
external fork PRs do not receive it. Inherited workflows that depend on PX4's
AWS runners remain enabled, but cannot run without that infrastructure.
