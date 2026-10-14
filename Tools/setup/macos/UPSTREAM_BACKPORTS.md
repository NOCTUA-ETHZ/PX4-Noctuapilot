# Upstream provenance of the macOS simulation changes

Reviewed on 2026-10-11 against official PX4 snapshots:

- `release/1.18`: `0299ef71a2a6fbb0d8cb3cfd25cf32cdda28a5d3`
- `main`: `bf6603b42b22bf69c3c187a8afcf2c42469e9932`

The consolidated `fix(simulation): backport upstream Gazebo and macOS setup
fixes` commit contains the selected source backports. Subsequent commits keep
local compatibility changes and Noctua telemetry features separate. The newer
flight stack is not merged: this branch retains its uint16_t actuator outputs.

## Adopted upstream implementations

| Change | Upstream source | Adaptation on this branch |
| --- | --- | --- |
| Motor/wheel Protobuf list construction | [013797605f / PR #27565](https://github.com/PX4/PX4-Autopilot/commit/013797605f8126a2f918b722e1963b35de753bba) | Uses upstream Clear/Reserve/Add sequence while retaining this branch's actuator interface. |
| Explicit servo and moving-platform numeric conversions | [2297e66a9d](https://github.com/PX4/PX4-Autopilot/commit/2297e66a9d4417b3ff27e3165650f99acd2a4bf7) | Both exact conversion hunks included in the consolidated commit; the moving-platform source was verified against official upstream. Intermediate gimbal changes excluded. |
| GStreamer camera linker directories | [059da2fcf7](https://github.com/PX4/PX4-Autopilot/commit/059da2fcf7e88fdae8e0373772032e47f383ff0e) | Exact five-line library-directory hunk, verified against official upstream main and release/1.18 snapshots; included in the consolidated commit. |
| Explicit gimbal numeric conversion | [54866b886e / PR #25770](https://github.com/PX4/PX4-Autopilot/commit/54866b886eabef69991b3e770c9032bb31867155) | Keeps this branch's Double message and adopts the final upstream conversion. |
| Protobuf-generated warning handling | [039ac8c4c1 / PR #26548](https://github.com/PX4/PX4-Autopilot/commit/039ac8c4c16e282446bc29040612786d808645bf) | Upstream target-scoped warning setting; handwritten PX4 sources keep their strict warnings. |
| Preserve shell variables and Homebrew runtime library lookup | [9c2e634325](https://github.com/PX4/PX4-Autopilot/commit/9c2e6343258094bc22dc9cc10b3b504493f7a127) | Adopts configure_file @ONLY and Darwin runtime lookup. |
| Locked Gazebo environment and activation before version check | [a8b0e91ebe](https://github.com/PX4/PX4-Autopilot/commit/a8b0e91ebe0f7f1f4c405bf566f72ecdcb523832) | Manifest and lock match official main byte for byte. Setup/runtime portions are selectively ported; the top-level automatic CMake prefix hint and CI changes are not imported. |
| OpticalFlow in-tree submodule build | [b911d4414e / PR #27184](https://github.com/PX4/PX4-Autopilot/commit/b911d4414e7c0915b4c581acbe4cbd21bc940ea1) | Plugin CMake file copied byte for byte from the reviewed 1.18 snapshot. Gitlink pinned to that snapshot's a4d4cfde0c023be6b1ff8c8e796862413bc99272, including its nested tracker. The previous ExternalProject file is removed. |
| OpticalFlow unused fields | [f32c3024b8](https://github.com/PX4/PX4-Autopilot/commit/f32c3024b832e2748476845d0ea3959930668ded) | Exact upstream header hunk. |
| OpticalFlow focal-length conversion | [fd1ddbc22e / PR #28715](https://github.com/PX4/PX4-Autopilot/commit/fd1ddbc22e31e7dd9718ecf686b67a065f586374) | Exact upstream main hunk. This fix is not in the reviewed release/1.18 snapshot. |

| Gazebo Classic compatibility with extended gimbal capabilities | [f835e077d0 / PR #1090](https://github.com/PX4/PX4-SITL_gazebo-classic/commit/f835e077d06eaf09a57d5152fcfb85244b53b77a) | Submodule advanced to the exact revision pinned by upstream `release/1.18`. Includes seven upstream commits since this branch's previous pin: MAVLink v2 cleanup, visualization fixes, configurable dialect support, first-message delivery and the missing gimbal `cap_flags2` argument. The companion `MAVLINK_DEVELOPMENT=ON` build argument matches upstream 1.18 and preserves the old development-header selection. No local Classic source patch. |

The locked environment is present in official main but absent from the reviewed
release/1.18 snapshot. The latter still installs simulation packages with
Homebrew. Therefore this is not exclusively a set of 1.18 backports.

## Local compatibility changes

The local Protobuf adjustment prefers the config package, uses
`protobuf_generate` and links `protobuf::libprotobuf`. The imported target
propagates Protobuf's dependency libraries and required C++ standard to
Gazebo consumers. Both reviewed upstream snapshots still use the older
Protobuf generation/link pattern, so this change is not an upstream backport.

Configure with `CMAKE_POLICY_VERSION_MINIMUM=3.5` when using CMake 4 with older
dependency build files. Checkout paths should have no spaces; no custom
source-path escaping or detached Gazebo environment is needed.

## Future upgrades

Compare each listed source patch with the chosen upstream base and drop
backports already present there. Reassess the local Protobuf adjustment and
the environment lock against that base. Keep the manifest and lock together;
do not update the package lock as part of an ordinary build.

## Validation

The macOS Gazebo SITL build, OpticalFlow plugin/library and nested tracker,
generated Protobuf messages, moving-platform plugin and GStreamer camera
plugin have compiled and linked successfully with strict PX4 warnings.
The checked-in OpticalFlow gitlink matches its clean dependency checkout.

The Noctua display demo has a separate isolated uORB-to-MAVLink smoke test;
see its README and the PR validation results for current checks. Host-library
build checks do not exercise an attached Pixhawk or a flying vehicle. The
latest `flash_v1.17` CAN decoding remains part of the base branch, rather than
being duplicated in the Noctua integration commits.
