# Two-Wheel Balancer RL

RL controller for a two-wheeled inverted pendulum robot (Altoids tin chassis, BLDC outrunners, Arduino Nano 33 BLE). Trains a PPO policy in MuJoCo simulation using [mjlab](https://github.com/mujocolab/mjlab) and deploys to the microcontroller.

Significant reference to [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie), which solves a very similar problem.

## Project structure

```
rl/
  src/two_wheel_mjlab/
    __init__.py                  # Package root (task registration, populated in Step 3)
    tasks/
      __init__.py                # mjlab task registry entry point
      balance_env_cfg.py         # (TODO) Isaac Lab-style manager env config
    robot/two_wheel/
      two_wheel.xml              # MJCF model of the robot
  sim.py                         # CPU playback: loads ONNX checkpoint, runs in mujoco.viewer
  tests/
    test_sim.py                  # Physics sanity tests for the MJCF model
  pyproject.toml
  rl_inverted_pendulum_research.md  # Full implementation plan and design notes
```

## Setup

Requires Python 3.10+ and [uv](https://docs.astral.sh/uv/).

```bash
cd rl/
uv sync
```

## Running tests

```bash
uv run pytest tests/ -v
```

Tests verify the MJCF model loads correctly, has the expected structure and mass budget, wheels contact the floor at rest, and basic physics behaves as expected (robot falls without torque, torque opposes fall).

## Running the simulator -- NOT FUNCTIONAL YET

Requires a trained ONNX checkpoint and a display (GLX/EGL):

```bash
uv run python sim.py --onnx-model-path <path_to_checkpoint.onnx>
```

Add `--delay` to simulate a 2-step sensor observation delay.
