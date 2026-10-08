#!/usr/bin/env python3
"""Host regression test using the pinned libuavcan dispatcher and real bridge.

Only PX4 parameters, clock and uORB publication are stubbed. No CAN hardware is
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

with tempfile.TemporaryDirectory(prefix="ie-fuelcell-test-") as work:
    work = Path(work)
    generated = work / "dsdl"
    subprocess.run([sys.executable, str(lib / "dsdl_compiler/libuavcan_dsdlc"),
                    "--outdir", str(generated), str(lib.parent / "dsdl/uavcan")], check=True)
    fields = []
    types = {"uint64": "uint64_t", "uint32": "uint32_t", "uint8": "uint8_t",
             "float32": "float", "bool": "bool"}
    for line in (repo / "msg/FuelCellCan.msg").read_text().splitlines():
        parts = line.split("#", 1)[0].split()
        if parts:
            field_type, name = parts
            if "[" in field_type:
                field_type, size = field_type.rstrip("]").split("[")
                name += f"[{int(size)}]"
            fields.append(f"{types[field_type]} {name};")
    stubs = {
        "uORB/topics/fuel_cell_can.h":
            "#pragma once\n#include <cstdint>\nstruct fuel_cell_can_s {\n"
            + "\n".join(fields) + "\n};\n",
        "uORB/Publication.hpp": """#pragma once
#include <uORB/topics/fuel_cell_can.h>
extern unsigned test_publications;
extern fuel_cell_can_s test_last;
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
#define PX4_ERR(...) do { printf(__VA_ARGS__); puts(""); } while (0)
""",
    }
    for name, content in stubs.items():
        target = work / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    # Core libuavcan sources: protocol/ needs generated DSDL and is not used.
    sources = [p for p in (lib / "src").rglob("*.cpp") if "protocol" not in p.parts]
    assert sources, "Initialize src/drivers/uavcan/libuavcan before running"
    command = [os.environ.get("CXX", "g++"), "-std=c++14", "-g", "-O1",
               "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-Wno-deprecated-copy",
               "-DUAVCAN_CPP_VERSION=2003", "-DUAVCAN_NO_ASSERTIONS",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "-I" + str(work), "-I" + str(generated), "-I" + str(here.parent),
               "-I" + str(lib / "include"), str(here / "test.cpp"),
               str(here.parent / "Bridge.cpp"), *map(str, sources),
               "-o", str(work / "test")]
    subprocess.run(command, check=True)
    subprocess.run([str(work / "test")], check=True)
