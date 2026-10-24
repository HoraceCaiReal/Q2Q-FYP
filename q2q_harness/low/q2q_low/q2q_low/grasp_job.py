"""Refresh the stationary world's scene before invoking the existing grasp flow."""
import subprocess
import sys
import signal
from pathlib import Path

if __name__ == '__main__':
    # The group's grasp child handles SIGTERM, cancels its action and writes proof.
    # Keep this supervisor alive until that child has completed cleanup.
    signal.signal(signal.SIGTERM, lambda signum, frame: None)
    loader = Path.home()/'ros2_ws/src/robot_moveit/scripts/scene_loader.py'
    code = subprocess.call(['python3', str(loader), '--world', sys.argv[1]])
    if code:
        raise SystemExit(code)
    raise SystemExit(subprocess.call(sys.argv[2:]))
