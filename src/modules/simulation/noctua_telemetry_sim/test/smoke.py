#!/usr/bin/env python3
"""Exercise Noctua uORB -> MAVLink using an isolated, non-flying PX4 instance."""

import argparse
import importlib.util
import math
import os
from pathlib import Path
import select
import shlex
import socket
import subprocess
import sys
import tempfile
import time


PUBLIC = "NOCTUA_FUEL_CELL_STATUS"
CAN = "NOCTUA_FUEL_CELL_CAN_DIAGNOSTICS"
GAS = "NOCTUA_GAS_SENSOR"
TYPES = {PUBLIC, CAN, GAS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--mavlink-dir", type=Path, required=True)
    args = parser.parse_args()
    build = args.build_dir.resolve()
    mavlink = args.mavlink_dir.resolve()
    executable = build / "bin/px4"
    if not executable.is_file():
        parser.error(f"Build PX4 first: {executable} is missing")

    with (
        tempfile.TemporaryDirectory(prefix="noctua-sim-smoke-") as tmp,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
    ):
        work = Path(tmp)
        generated = work / "noctua_wire.py"
        subprocess.run(
            [
                sys.executable,
                str(mavlink / "pymavlink/tools/mavgen.py"),
                "--lang=Python3",
                "--wire-protocol=2.0",
                f"--output={generated}",
                str(mavlink / "message_definitions/v1.0/noctua.xml"),
            ],
            check=True,
            env={**os.environ, "PYTHONHASHSEED": "0"},
            stdout=subprocess.DEVNULL,
        )
        spec = importlib.util.spec_from_file_location("noctua_wire", generated)
        dialect = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(dialect)
        decoder = dialect.MAVLink(None)
        receiver.bind(("127.0.0.1", 0))
        receiver_port = receiver.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            sender_port = reservation.getsockname()[1]
        instance = 100 + os.getpid() % 1000
        startup = work / "startup.sh"
        startup.write_text(
            "#!/bin/sh\nset -e\n"
            f". {shlex.quote(str(build / 'bin/px4-alias.sh'))}\n"
            "dataman start\n"
            "simulator_sih start\n"
            "noctua_telemetry_sim start\n"
            f"mavlink start -u {sender_port} -o {receiver_port} -t 127.0.0.1 -r 100000\n"
            "mavlink boot_complete\n"
        )

        def command(*arguments, expected=0, module="noctua_telemetry_sim"):
            result = subprocess.run(
                [
                    str(build / f"bin/px4-{module}"),
                    "--instance",
                    str(instance),
                    *arguments,
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            assert result.returncode == expected, result.stdout + result.stderr
            return result.stdout + result.stderr

        def collect(duration):
            messages = []
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                ready, _, _ = select.select(
                    [receiver], [], [], max(0, deadline - time.monotonic())
                )
                if ready:
                    packet, _ = receiver.recvfrom(65535)
                    messages.extend(
                        m
                        for m in decoder.parse_buffer(packet) or []
                        if m.get_type() in TYPES
                    )
            return messages

        def wait_for(predicate, timeout=10):
            messages = []
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                messages.extend(collect(0.25))
                if predicate(messages):
                    return messages
            raise AssertionError("Timed out waiting for Noctua MAVLink samples")

        with (work / "px4.log").open("w+") as log:
            process = subprocess.Popen(
                [
                    str(executable),
                    "-d",
                    "-i",
                    str(instance),
                    "-s",
                    str(startup),
                    "-w",
                    str(work),
                    str(build / "etc"),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                samples = wait_for(lambda ms: TYPES <= {m.get_type() for m in ms})
                public = next(m for m in samples if m.get_type() == PUBLIC)
                can = next(m for m in samples if m.get_type() == CAN)
                gas = next(m for m in samples if m.get_type() == GAS)
                assert public.source == 1 and public.connected == 1
                assert 80 < public.hydrogen_remaining_pct <= 82
                assert 0.7 < public.regulated_pressure_bar < 0.9
                assert math.isnan(public.tank_pressure_bar)
                assert public.output_power > 0 and public.battery_power < 0
                assert public.raw_can_error == -1
                assert public.main_error == public.sub_error == 0
                assert can.connected == 1 and can.can_interface == 1
                assert can.tank_pressure_bar > 200 and can.received_frames > 0
                assert can.raw_data[0] >> 4 == can.counter
                assert can.raw_data[0] & 15 == can.state
                assert gas.gas_type == 5 and gas.device_id & 7 == 4  # H2, simulated bus
                assert 0.7 < gas.concentration < 1.7 and 22 < gas.temperature < 26
                assert all(
                    m.sample_time_usec > 0 and m.time_usec >= m.sample_time_usec
                    for m in (public, can, gas)
                )
                changing = wait_for(
                    lambda ms: any(
                        m.get_type() == PUBLIC
                        and m.hydrogen_remaining_pct < public.hydrogen_remaining_pct
                        for m in ms
                    )
                )
                assert any(m.get_type() == PUBLIC for m in changing)
                print(
                    "PASS: changing UART fuel-cell data, CAN diagnostics and H2 probe cross MAVLink"
                )

                command("source", "can")
                samples = wait_for(
                    lambda ms: (
                        any(m.get_type() == PUBLIC and m.source == 2 for m in ms)
                        and any(m.get_type() == CAN for m in ms)
                    )
                )
                public = next(
                    m for m in samples if m.get_type() == PUBLIC and m.source == 2
                )
                assert public.connected == 1 and public.tank_pressure_bar > 200
                assert (
                    public.raw_can_error == 0
                    and public.main_error == public.sub_error == -1
                )
                assert math.isnan(public.hydrogen_remaining_pct)
                assert math.isnan(public.regulated_pressure_bar)
                assert public.output_power > 0 and public.stack_power > public.output_power
                assert public.battery_power < 0
                can = next(m for m in samples if m.get_type() == CAN)
                assert can.output_power > 0 and can.battery_power < 0
                print(
                    "PASS: CAN mode publishes validated powers and preserves unavailable UART fields"
                )

                command("pause")
                collect(1)  # Drain samples already in flight before pause.
                assert not collect(2), "Paused simulator still publishes telemetry"
                command("source", "uart")
                command("resume")
                wait_for(
                    lambda ms: (
                        TYPES <= {m.get_type() for m in ms}
                        and any(m.get_type() == PUBLIC and m.source == 1 for m in ms)
                    )
                )
                command("stop")
                command("status", expected=255)
                collect(1)
                assert not collect(2), "Stopped simulator still publishes telemetry"
                print("PASS: pause, source switching, resume and clean shutdown")
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                # Only terminate the process started by this test; existing SITL/Gazebo stays untouched.
                if process.poll() is None:
                    for module, action in (
                        ("noctua_telemetry_sim", "stop"),
                        ("mavlink", "stop-all"),
                        ("simulator_sih", "stop"),
                        ("dataman", "stop"),
                    ):
                        try:
                            command(action, module=module)
                        except (AssertionError, subprocess.SubprocessError):
                            pass
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


if __name__ == "__main__":
    main()
