"""Standalone smoke test for the MPC core without ROS 2.

This script is useful if you want to quickly verify the controller logic before
running the full ROS 2 workspace.
"""

import sys
from pathlib import Path
import numpy as np

repo_root = Path(__file__).resolve().parent
pkg_path = repo_root / 'src' / 'drone_mpc_ros'
if str(pkg_path) not in sys.path:
    sys.path.insert(0, str(pkg_path))

from drone_mpc_ros.mpc_core import DroneMPC


def main() -> None:
    controller = DroneMPC()
    x = np.zeros(6)
    xref = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])

    dt = 0.1
    drag = 0.15

    print('Running a 30-step standalone MPC smoke test...')
    for k in range(30):
        u = controller.solve(x, xref)
        accel = u - drag * x[3:6]
        x[3:6] += dt * accel
        x[0:3] += dt * x[3:6]
        print(f"step={k:02d} pos={x[0:3]} vel={x[3:6]} cmd={u}")


if __name__ == '__main__':
    main()
