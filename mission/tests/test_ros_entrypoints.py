import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix


RUNTIME_ROS = Path(__file__).resolve().parents[1] / "runtime_ros"


def test_ros_executables_use_the_jazzy_system_python():
    for path in sorted(RUNTIME_ROS.glob("*.py")):
        if path.name in {"__init__.py", "no_motion_backend.py"}:
            continue
        source = path.read_bytes()
        assert source.splitlines()[0] == b"#!/usr/bin/python3"
        assert b'if __name__ == "__main__":' in source


def test_installed_excavation_cycle_server_starts_without_source_tree_on_pythonpath():
    prefix = Path(get_package_prefix("airy_mission_runtime"))
    executable = (
        prefix / "lib" / "airy_mission_runtime" / "excavation_cycle_server"
    )
    environment = dict(os.environ)
    source_root = str(Path(__file__).resolve().parents[2])
    environment["PYTHONPATH"] = os.pathsep.join(
        entry
        for entry in environment.get("PYTHONPATH", "").split(os.pathsep)
        if entry and Path(entry).resolve() != Path(source_root).resolve()
    )
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "orin_host:=127.0.0.1",
            "-p",
            "orin_port:=18083",
        ],
        cwd="/",
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        time.sleep(0.75)
        assert process.poll() is None, process.stdout.read()
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5.0)
