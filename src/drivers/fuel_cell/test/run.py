#!/usr/bin/env python3
import os
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
repo = here.parents[3]
with tempfile.TemporaryDirectory(prefix="fuel-cell-test-") as directory:
    work = Path(directory)
    types = {"uint64": "uint64_t", "uint32": "uint32_t", "int32": "int32_t",
             "uint8": "uint8_t", "float32": "float", "bool": "bool"}
    for message, topic in [("FuelCell", "fuel_cell"), ("FuelCellCan", "fuel_cell_can")]:
        fields = []
        for line in (repo / "msg" / (message + ".msg")).read_text().splitlines():
            parts = line.split("#", 1)[0].split()
            if not parts:
                continue
            kind, name = parts
            if "[" in kind:
                kind, size = kind.rstrip("]").split("[")
                name += f"[{int(size)}]"
            fields.append(f"{types[kind]} {name};")
        path = work / "uORB/topics" / (topic + ".h")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("#pragma once\n#include <cstdint>\nstruct " + topic + "_s {\n"
                        + "\n".join(fields) + "\n};\n")
    executable = work / "test"
    subprocess.run([os.environ.get("CXX", "g++"), "-std=c++14", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", "-g", "-I" + str(work), "-I" + str(here.parent),
                    str(here / "test.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
