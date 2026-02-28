"""Launch an interactive MuJoCo viewer for the two-wheel robot model.

Usage:
    LIBGL_ALWAYS_SOFTWARE=1 uv run python scripts/view_model.py

    # Drop physics so it holds in the upright pose for inspection:
    LIBGL_ALWAYS_SOFTWARE=1 uv run python scripts/view_model.py --paused
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

XML_PATH = Path(__file__).parents[1] / "src/two_wheel_mjlab/robot/two_wheel/two_wheel.xml"


def load_scene(xml_path: Path = XML_PATH) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load the robot model and initialise a data object at the resting pose."""
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--paused", action="store_true",
        help="Start with physics paused so the robot holds its upright pose.",
    )
    parser.add_argument(
        "--xml", type=Path, default=XML_PATH,
        help="Path to the MJCF file (default: two_wheel.xml).",
    )
    args = parser.parse_args()

    model, data = load_scene(args.xml)

    print(f"Model: {args.xml.name}")
    print(f"  Bodies: {model.nbody}  Joints: {model.njnt}  Actuators: {model.nu}")
    print(f"  Total mass: {model.body_mass.sum() * 1000:.1f} g")
    print()
    if args.paused:
        print("Physics paused — step manually with the spacebar.")
    else:
        print("Physics running — robot will fall without control torque.")
    print("Tip: if the window fails to open, prefix with LIBGL_ALWAYS_SOFTWARE=1")

    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.distance = 0.50
        v.cam.elevation = -15
        v.cam.azimuth = 135
        if args.paused:
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True

        while v.is_running():
            if not args.paused:
                mujoco.mj_step(model, data)
            v.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
