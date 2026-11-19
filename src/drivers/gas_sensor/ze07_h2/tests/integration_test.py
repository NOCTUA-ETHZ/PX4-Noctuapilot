#!/usr/bin/env python3
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise the real driver/uORB through a pseudo-terminal in a minimal PX4 host build.
Usage: python3 integration_test.py /absolute/path/to/build/bin/px4
Requires param, listener and ze07_h2 in that build; no simulator/hardware needed.
"""
import os
from pathlib import Path
import pty
import re
import select
import subprocess
import sys
import tempfile
import termios
import time

binary = Path(sys.argv[1]).resolve()
instance = str(os.getpid())
master, slave = pty.openpty()
console, console_slave = pty.openpty()
sequence = 0
transcript = []


def frame(ppm):
    data = bytearray([255, 6, 3, 0, ppm >> 8, ppm & 255, 0x75, 0x30, 0])
    data[8] = -sum(data[1:8]) & 255
    return data


def read_until(marker, timeout=8):
    data = b""
    deadline = time.monotonic() + timeout
    while marker not in data:
        remaining = deadline - time.monotonic()
        assert remaining > 0, data.decode(errors="replace")
        if select.select([console], [], [], remaining)[0]:
            data += os.read(console, 65536)
    text = re.sub(r"\[[0-9;]*[A-Za-z]", "", data.decode(errors="replace"))
    transcript.append(text)
    return text


def command(module, *args, check=True):
    global sequence
    sequence += 1
    sentinel = "__ze07_done_" + str(sequence)
    os.write(console, (" ".join([module, *args]) + "\n" + sentinel + "\n").encode())
    text = read_until(("Invalid command: " + sentinel).encode())
    if check:
        assert "failed, returned" not in text, text
    return text


def sample():
    text = command("listener", "sensor_gas_concentration", "-n", "1")
    timestamp = re.search(r"timestamp_sample: (\d+)", text)
    assert timestamp, text
    return timestamp[1], text


with tempfile.TemporaryDirectory(prefix="ze07-test-") as directory:
    startup = Path(directory) / "rcS"
    startup.write_text("#!/bin/sh\nexit 0\n")
    process = subprocess.Popen([str(binary), "-i", instance, "-s", str(startup),
                                "-w", directory], stdin=console_slave, stdout=console_slave, stderr=console_slave)
    try:
        read_until(b"pxh>")
        command("param", "touch", "SENS_ZE07_WARM")
        assert re.search(r"SENS_ZE07_WARM \[.*?\] : 300", command("param", "show", "SENS_ZE07_WARM"))
        command("param", "select", str(Path(directory) / "parameters.bson"))
        command("param", "set", "SENS_ZE07_WARM", "1")
        command("ze07_h2", "start", "-d", os.ttyname(slave))
        attrs = termios.tcgetattr(slave)
        assert attrs[4] == attrs[5] == termios.B9600
        assert attrs[2] & termios.CSIZE == termios.CS8
        assert not attrs[2] & (termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        assert select.select([master], [], [], 2)[0], "Missing upload-mode command"
        assert os.read(master, 9) == bytes([255, 1, 0x78, 0x40, 0, 0, 0, 0, 0x47])
        os.write(master, frame(100))
        time.sleep(0.1)
        assert "waiting for valid data / warming up" in command("ze07_h2", "status")
        time.sleep(1.05)
        packet = frame(1234)
        os.write(master, packet[:4])
        time.sleep(0.04)
        os.write(master, packet[4:])
        time.sleep(0.1)
        stamp, text = sample()
        assert "concentration_ppm: 1234" in text, text
        assert "temperature: nan" in text.lower(), text
        assert "gas_type: 5" in text, text
        assert "fresh" in command("ze07_h2", "status")
        bad = frame(9000)
        bad[-1] ^= 1
        os.write(master, bad)
        time.sleep(0.1)
        assert sample()[0] == stamp, "Published corrupt data"
        # Partial packet separated by >200 ms must not form a measurement.
        os.write(master, packet[:4])
        time.sleep(0.3)
        os.write(master, packet[4:])
        time.sleep(0.1)
        assert sample()[0] == stamp, "Combined frames across a timeout"
        os.write(master, frame(500))
        time.sleep(0.1)
        stamp, text = sample()
        assert "concentration_ppm: 500" in text, text
        time.sleep(5.2)
        assert "stale/warming up" in command("ze07_h2", "status")
        assert sample()[0] == stamp, "Republished stale data"
        os.write(master, frame(750))
        time.sleep(0.1)
        assert sample()[0] == stamp, "Skipped warm-up after outage"
        time.sleep(1.05)
        os.write(master, frame(750))
        time.sleep(0.1)
        assert "concentration_ppm: 750" in sample()[1]
        command("ze07_h2", "stop")
        # Restart and clean stop also exercise publication lifetime/UART close.
        command("ze07_h2", "start", "-d", os.ttyname(slave))
        command("ze07_h2", "stop")
        result = command("ze07_h2", "start", "-d", "/not/a/serial/device", check=False)
        assert "cannot configure" in result, result
        print("ZE07-H2 PX4 UART/uORB integration tests passed")
    except BaseException:
        print("\n".join(transcript), file=sys.stderr)
        raise
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)
        os.close(console)
        os.close(console_slave)
