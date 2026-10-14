# Noctua telemetry demo

`px4_sitl_noctua` adds a SITL-only publisher to standard PX4 SITL. It publishes
the actual `fuel_cell`, `fuel_cell_can` and `sensor_gas_concentration` uORB topics.
The Noctua MAVLink streams transport these to noctua-GC exactly as on hardware.
The module is excluded from standard builds and rejects hardware build targets.
Publishing still requires explicit startup, even in the Noctua build.

This is a display and integration demo, not a physical model of hydrogen
consumption, leakage, electrical performance or safety behavior. It supplies
changing sample values and does not alter Gazebo physics, battery simulation,
flight control or failsafes. Simulated demand increases when PX4 is armed.

## Build on this Mac

Reuse the existing integration build directory. Unsetting `PX4_CONFIG_FILE`
lets PX4 select the new board configuration instead of its cached default.

```sh
source .venv/bin/activate
source .venv/bin/px4-gz-env.sh
export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer
export GZ_DISTRO=harmonic
ulimit -S -n 2048

cmake -S . -B build/px4_sitl_noctua -UPX4_CONFIG_FILE \
  -DCONFIG=px4_sitl_noctua -UNOCTUA_MAVLINK_SOURCE_DIR
cmake --build build/px4_sitl_noctua --parallel 4
```

For a fresh build directory, also specify `-G Ninja`,
`-DPYTHON_EXECUTABLE="$PWD/.venv/bin/python"`,
`-DCMAKE_PREFIX_PATH="$CONDA_PREFIX"` and
`-DCMAKE_POLICY_VERSION_MINIMUM=3.5` as in the macOS setup instructions.

## Run with Gazebo and noctua-GC

Stop your previous SITL/Gazebo session and close any stock QGC instance first.

```sh
open ../noctua-GC/build/Debug/noctua-GC.app
PX4_NOCTUA_TELEMETRY=1 cmake --build build/px4_sitl_noctua --target gz_x500
```

The startup log explicitly reports simulated fuel-cell and H2-probe data.
By default, the public fuel-cell topic simulates UART: about 82% hydrogen,
0.8 bar inlet pressure, 48 V, 200 W idle output and negative battery power.
H2 probe concentration oscillates around 1.2 ppm, temperature around 24 °C.
CAN diagnostics also publish a synthetic payload, tank pressure, counters,
voltage and raw decoded power. All timestamps use PX4's simulation clock.

Use these commands in the PX4 `pxh>` prompt or noctua-GC's MAVLink Console:

```sh
noctua_telemetry_sim status
noctua_telemetry_sim source can
noctua_telemetry_sim source uart
noctua_telemetry_sim pause
noctua_telemetry_sim resume
noctua_telemetry_sim stop
```

CAN mode reuses the real driver's mapping: tank gauge pressure is available;
UART percentage and regulated inlet pressure are unavailable. Output power,
SPM output power and signed battery power use the driver's validated MCU v3.63
mapping. Raw CAN payloads remain in diagnostics. Source changes do not affect
the H2 probe.
Pause stops every sample stream so QGC shows lost telemetry after five seconds;
resume restores samples. Stop removes the publisher. Without the environment
flag, you can start it manually with `noctua_telemetry_sim start`.

## Verify the wire transport

`test/smoke.py` starts its own PX4 instance in a temporary directory, without
Gazebo or flight commands, and receives MAVLink on an isolated loopback port.
PX4's headless SIH simulator advances the simulation clock for this test.
It checks changing UART samples, raw diagnostics, H2 probe values, CAN-mode
unavailable semantics, pause/resume and clean module shutdown.

```sh
python src/modules/simulation/noctua_telemetry_sim/test/smoke.py \
  --build-dir build/px4_sitl_noctua \
  --mavlink-dir src/modules/mavlink/mavlink
```

The test starts its own PX4 instance and never arms or commands a vehicle.

Repeat it with `--mavlink-mode onboard` to cover the default USB telemetry
profile. Both modes must deliver fuel-cell status, CAN diagnostics and gas
samples without manual stream requests.
