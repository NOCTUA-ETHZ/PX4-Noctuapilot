#!/usr/bin/env python3
"""Host regression test using the pinned libuavcan dispatcher and real bridge.

PX4 parameters, clock, device plumbing and uORB publication are stubbed. No CAN hardware is
required. Initialize libuavcan recursively and install its Python dependencies.
"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

here = Path(__file__).resolve().parent
repo = here.parents[4]
lib = here.parents[1] / "libdronecan/libuavcan"

with tempfile.TemporaryDirectory(prefix="dronecan-rc-test-") as work:
    work = Path(work)
    generated = work / "dsdl"
    subprocess.run([sys.executable, str(lib / "dsdl_compiler/libuavcan_dsdlc"),
                    "--outdir", str(generated), str(lib.parent / "dsdl/uavcan"), str(lib.parent / "dsdl/dronecan")], check=True)
    fields = []
    types = {"uint64": "uint64_t", "uint32": "uint32_t", "uint16": "uint16_t",
             "uint8": "uint8_t", "int8": "int8_t", "int32": "int32_t",
             "float32": "float", "bool": "bool"}
    for line in (repo / "msg/InputRc.msg").read_text().splitlines():
        parts = line.split("#", 1)[0].split()
        if not parts:
            continue
        field_type, name = parts[:2]
        if len(parts) == 4 and parts[2] == "=":
            fields.append(f"static constexpr {types[field_type]} {name} = {parts[3]};")
            continue
        if "[" in field_type:
            field_type, size = field_type.rstrip("]").split("[")
            name += f"[{int(size)}]"
        fields.append(f"{types[field_type]} {name};")
    stubs = {
        "uORB/topics/input_rc.h":
            "#pragma once\n#include <cstdint>\nstruct input_rc_s {\n"
            + "\n".join(fields) + "\n};\n",
        "uORB/Publication.hpp": """#pragma once
#include <uORB/topics/input_rc.h>
extern unsigned test_publications;
extern input_rc_s test_last;
#define ORB_ID(name) 0
namespace uORB {
template<class T> class Publication {
public:
    explicit Publication(int) {}
    bool publish(const T& sample) { test_last = sample; ++test_publications; return true; }
};
}
""",
        "drivers/drv_hrt.h": "#pragma once\n#include <cstdint>\nuint64_t hrt_absolute_time();\n",
        "parameters/param.h": "#pragma once\nint param_find(const char*);\nint param_get(int, void*);\n",
        "px4_platform_common/log.h": """#pragma once
#include <cstdio>
#define PX4_INFO(...) do { printf(__VA_ARGS__); puts(""); } while (0)
#define PX4_WARN(...) do { printf(__VA_ARGS__); puts(""); } while (0)
#define PX4_ERR(...) do { printf(__VA_ARGS__); puts(""); } while (0)
""",
    }
    stubs.update({
        "px4_platform_common/defines.h": "#pragma once\n",
        "drivers/drv_orb_dev.h": "#pragma once\n#define PX4_OK 0\n",
        "uORB/uORB.h": "#pragma once\n\n#define ORB_ID(name) 0\nusing orb_id_t = const void*;\nusing orb_advert_t = void*;\n",
        "lib/drivers/device/Device.hpp": """#pragma once
namespace device {
class Device {
public:
    enum DeviceBusType { DeviceBusType_UAVCAN };
    explicit Device(const char*) {}
    void set_device_bus_type(DeviceBusType) {}
    void set_device_bus(int) {}
};
}
""",
    })
    topics = {
        "battery_status": "uint64_t timestamp; bool connected; float voltage_v, current_a, discharged_mah, remaining;",
        "sensor_gps": "uint64_t timestamp; uint8_t fix_type, satellites_used; double latitude_deg, longitude_deg; float altitude_msl_m, vel_m_s, cog_rad;",
        "vehicle_attitude": "uint64_t timestamp; float q[4];",
        "vehicle_status": "uint64_t timestamp; uint8_t nav_state; " + " ".join(
            f"static constexpr uint8_t NAVIGATION_STATE_{name} = {i};" for i, name in enumerate(
                ["MANUAL", "ALTCTL", "POSCTL", "AUTO_MISSION", "AUTO_RTL", "AUTO_LOITER", "OFFBOARD", "STAB", "ACRO"]))}
    subscriptions = "#pragma once\n#include <uORB/topics/input_rc.h>\nextern input_rc_s test_last;\n"
    for topic, fields in topics.items():
        stubs[f"uORB/topics/{topic}.h"] = f"#pragma once\n#include <cstdint>\nstruct {topic}_s {{ {fields} }};\n"
        subscriptions += f"#include <uORB/topics/{topic}.h>\nextern {topic}_s test_{topic};\n"
    subscriptions += "namespace uORB { class Subscription { public: explicit Subscription(int) {}\n"
    subscriptions += "bool copy(input_rc_s *v) { *v = test_last; return true; }\n"
    for topic in topics:
        subscriptions += f"bool copy({topic}_s *v) {{ *v = test_{topic}; return true; }}\n"
    subscriptions += "}; }\n"
    stubs["uORB/Subscription.hpp"] = subscriptions
    for name, content in stubs.items():
        target = work / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    # Core libuavcan sources: protocol/ needs generated DSDL and is not used.
    sources = [p for p in (lib / "src").rglob("*.cpp") if "protocol" not in p.parts]
    assert sources, "Initialize src/drivers/uavcan/libdronecan submodules before running"
    command = [os.environ.get("CXX", "g++"), "-std=c++14", "-g", "-O1",
               "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-Wno-deprecated-copy",
               "-DUAVCAN_CPP_VERSION=2003", "-DUAVCAN_MEM_POOL_BLOCK_SIZE=128",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "-I" + str(work), "-I" + str(generated), "-I" + str(here.parent),
               "-I" + str(repo / "src/lib/matrix"), "-I" + str(lib / "include"), "-I" + str(repo / "src/include"), str(here / "test.cpp"),
               str(here.parent / "rc_input.cpp"), str(here.parent / "rc_telemetry.cpp"), *map(str, sources),
               "-o", str(work / "test")]
    subprocess.run(command, check=True)
    subprocess.run([str(work / "test")], check=True)
