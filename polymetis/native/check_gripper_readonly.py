"""Reject a motion-producing configuration before any robot connection."""

from pathlib import Path
import subprocess
import sys
import tempfile


with tempfile.TemporaryDirectory() as directory:
    config = Path(directory) / "invalid.yaml"
    # Deliberately omit robot_ip: the read-only guard must run before connecting.
    config.write_text(
        "control_ip: 127.0.0.1\ncontrol_port: 59999\n"
        "readonly: true\nhome_on_start: true\n"
    )
    result = subprocess.run([sys.argv[1], str(config)], capture_output=True, text=True, timeout=5)
    assert result.returncode == 1, result
    assert "Read-only gripper cannot home on startup" in result.stdout + result.stderr, result
