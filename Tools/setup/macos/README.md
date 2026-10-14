# Noctua PX4: Gazebo on macOS

This integration branch is based on `flash_v1.17`. Its Gazebo libraries come
from the exact package builds recorded in `pixi.lock`, verified against
official upstream main. Keep the manifest and lock file together in Git.
Homebrew supplies general build tools; it does not supply this Gazebo stack.

See [UPSTREAM_BACKPORTS.md](UPSTREAM_BACKPORTS.md) for the upstream source
commits, remaining local adaptations and validation limits. The locked setup
is from upstream main; the reviewed release/1.18 snapshot still uses Homebrew.

## First-time setup

With Xcode installed, run from the PX4 repository. This branch has been
configured with Python 3.10; the existing `.venv` uses that version:

```bash
./Tools/setup/macos.sh --sim-tools
```

The script installs Pixi, installs the locked Gazebo environment, and adds
its activation to `.venv/bin/activate`. Generated Gazebo packages use the
standard upstream location, `Tools/setup/macos/.pixi/envs/default`. The script
also installs the general PX4 build tools. An existing Python venv is reused.
Use a checkout path without spaces for new builds.

## Configure and build

Activate the environment in each new terminal. Use a new build directory
when moving from Homebrew Gazebo to the locked environment or renaming the
workspace; CMake caches absolute source and library paths. Run these commands
from the PX4 repository root.

```bash
source .venv/bin/activate
export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer
export GZ_DISTRO=harmonic
ulimit -S -n 2048

cmake -S . -B build/px4_sitl_locked -G Ninja \
  -DCONFIG=px4_sitl_default \
  -DCMAKE_MAKE_PROGRAM=/opt/homebrew/bin/ninja \
  -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python" \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5

cmake --build build/px4_sitl_locked --parallel 4
```

Gazebo, OpenCV, Protobuf, Abseil and GStreamer must resolve inside the
environment selected by `$CONDA_PREFIX`. Do not copy Homebrew package paths
from an older CMake cache into this build.

## Run

After a successful build:

```bash
open "../noctua-GC/build/Debug/noctua-GC.app"
cmake --build build/px4_sitl_locked --target gz_x500
```

## Reinstall or update the environment

To reproduce the checked-in versions without changing the lock:

```bash
pixi install --locked --manifest-path Tools/setup/macos/pixi.toml
```

Update the lock deliberately, then review and verify the resulting simulator
build before committing the new manifest and lock. Do not run an update as
part of a normal build. `.venv` and `.pixi` are local generated directories
and are ignored by Git.
