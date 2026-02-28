# RL Controller for Two-Wheeled Inverted Pendulum: Implementation Plan

**Robot:** Altoids tin chassis, two BLDC outrunners, SimpleFOC, magnetic encoder, Arduino Nano 33 BLE (nRF52840), onboard IMU.
**Goal:** Train a PPO policy in MuJoCo simulation using [mjlab](https://github.com/mujocolab/mjlab) and deploy it to the Arduino.
**Framework:** mjlab (Isaac Lab manager API + MuJoCo Warp, GPU-accelerated). Reference: [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie) — a nearly identical wheeled biped project.

> **mjlab is in beta.** Expect the API to evolve. Pin to a specific commit or release when you start. Install from source for latest fixes:
> ```bash
> uv add git+https://github.com/mujocolab/mjlab rsl-rl wandb mujoco
> ```

---

## TODO

- [ ] **Bootstrap** — clone mjlab_upkie to temp, copy scaffold into `rl/`, verify `import mjlab` works
- [ ] **Robot model** — write `two_wheel.xml`, verify visually with `mujoco.viewer`
- [ ] **Env config** — write `balance_env_cfg.py`, register `TwoWheel-Balance-v0`
- [ ] **Baseline training** — train without domain randomization, confirm episode length reaches 10s
- [ ] **Visualization** — run `sim.py` on a trained checkpoint, visually confirm balancing
- [ ] **Domain randomization** — add one parameter group at a time (friction → mass → damping → motor gain → IMU noise → action latency), retrain, confirm convergence each time
- [ ] **Weight export** — export weights + obs normalizer stats to `firmware/policy_weights.h` and `firmware/obs_norm.h`
- [ ] **C++ validation** — cross-check Python and C++ inference numerically on a set of test inputs before touching hardware
- [ ] **Arduino deployment** — flash, log raw IMU vs sim distributions, tune `MAX_TORQUE_NM`, enable RL policy

---

## Table of Contents

1. [Project Structure](#1-project-structure)
2. [Step 1: Bootstrap from mjlab_upkie](#2-step-1-bootstrap-from-mjlab_upkie)
3. [Step 2: MJCF Robot Model](#3-step-2-mjcf-robot-model)
4. [Step 3: Environment Configuration](#4-step-3-environment-configuration)
5. [Step 4: Train a Baseline Policy](#5-step-4-train-a-baseline-policy)
6. [Step 5: Evaluate and Visualize (CPU)](#6-step-5-evaluate-and-visualize-cpu)
7. [Step 6: Domain Randomization](#7-step-6-domain-randomization)
8. [Step 7: Export Weights and Deploy to Arduino](#8-step-7-export-weights-and-deploy-to-arduino)
9. [Key Gotchas](#9-key-gotchas)
10. [Sources](#10-sources)

---

## 1. Project Structure

```
rl/
├── src/
│   └── two_wheel_mjlab/
│       ├── __init__.py              # mjlab.register() call
│       ├── tasks/
│       │   └── balance_env_cfg.py   # Isaac Lab-style manager config
│       └── robot/
│           └── two_wheel/
│               └── two_wheel.xml    # MJCF model
├── sim.py                           # CPU-side playback via mujoco.viewer
├── export_weights.py                # RSL-RL checkpoint -> C header
├── pyproject.toml                   # uv project metadata
├── logs/                            # RSL-RL checkpoints (auto-created)
└── firmware/
    ├── policy_weights.h             # Generated weight arrays
    ├── obs_norm.h                   # Generated running mean/std
    └── policy_inference.ino         # Arduino sketch
```

mjlab_upkie has an essentially identical layout. Use it as the living reference.

---

## 2. Step 1: Bootstrap from mjlab_upkie

mjlab_upkie implements velocity tracking for the Upkie wheeled biped using mjlab — the task is close enough to use as a starting template. The robot model and env config both need to be swapped out, but `sim.py`, `pyproject.toml`, and the src layout are directly reusable.

Since `rl/` already lives inside a larger repo, **don't use submodules**. Clone mjlab_upkie to a temp location, copy the files you want, then discard the clone:

```bash
git clone https://github.com/MarcDcls/mjlab_upkie /tmp/mjlab_upkie

cp /tmp/mjlab_upkie/sim.py rl/
cp /tmp/mjlab_upkie/pyproject.toml rl/
cp -r /tmp/mjlab_upkie/src/mjlab_upkie/ rl/src/two_wheel_mjlab/

rm -rf /tmp/mjlab_upkie
```

Edit immediately after copying:
- `pyproject.toml`: rename the project, update the package name to `two_wheel_mjlab`
- `src/two_wheel_mjlab/`: delete Upkie's robot XML and task config — these will be replaced in Steps 2–3
- `sim.py`: update the task ID string to `TwoWheel-Balance-v0`

Everything lands in your normal git history with no submodule overhead. You won't get upstream mjlab_upkie fixes automatically, but since the robot and task are being replaced wholesale, that's a non-issue.

However, be sure to leave a reference in the readme and keep their licensing intact!

```bash
cd rl/
# Install dependencies (Python 3.10+, NVIDIA GPU for training)
uv sync

# Verify the copied scaffold installs cleanly before modifying anything
uv run python -c "import mjlab; print('mjlab ok')"
```

---

## 3. Step 2: MJCF Robot Model

File: `src/two_wheel_mjlab/robot/two_wheel/two_wheel.xml`

The MJCF is framework-agnostic — mjlab loads it the same way as plain `mujoco`. Model the real hardware as closely as possible; tune masses and geometry from physical measurements.

```xml
<mujoco model="two_wheel_balancer">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>

  <default>
    <joint damping="0.01" armature="0.01"/>
    <geom condim="3" friction="0.8 0.1 0.1"/>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
             rgb2=".2 .3 .4" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" diffuse="1 1 1"/>
    <geom name="floor" size="0 0 .05" type="plane" material="grid" condim="3"/>

    <body name="chassis" pos="0 0 0.065">
      <freejoint name="chassis_free"/>
      <!-- ~80g: altoids tin + battery + electronics -->
      <geom name="chassis_box" type="box" size="0.03 0.025 0.05"
            mass="0.08" rgba="0.6 0.3 0.1 1"/>
      <site name="imu_site" pos="0 0 0" size="0.005"/>

      <body name="left_wheel" pos="0 -0.04 -0.05">
        <joint name="left_wheel_joint" type="hinge" axis="0 1 0" damping="0.005"/>
        <geom type="cylinder" size="0.035 0.008" mass="0.02"
              rgba="0.2 0.2 0.8 1" friction="1.0 0.1 0.1"/>
      </body>

      <body name="right_wheel" pos="0 0.04 -0.05">
        <joint name="right_wheel_joint" type="hinge" axis="0 1 0" damping="0.005"/>
        <geom type="cylinder" size="0.035 0.008" mass="0.02"
              rgba="0.2 0.2 0.8 1" friction="1.0 0.1 0.1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- gear = torque scale; tune against measured SimpleFOC stall torque (~0.1–0.3 Nm) -->
    <motor name="left_motor"  joint="left_wheel_joint"
           ctrllimited="true" ctrlrange="-1.0 1.0" gear="1"/>
    <motor name="right_motor" joint="right_wheel_joint"
           ctrllimited="true" ctrlrange="-1.0 1.0" gear="1"/>
  </actuator>

  <sensor>
    <gyro      name="imu_gyro"       site="imu_site" noise="0.01"/>
    <framequat name="chassis_quat"   objtype="body" objname="chassis"/>
    <jointvel  name="left_wheel_vel" joint="left_wheel_joint"/>
    <jointvel  name="right_wheel_vel" joint="right_wheel_joint"/>
  </sensor>
</mujoco>
```

**Modeling decisions:**
- `freejoint` gives full 6-DOF chassis motion — pitch, roll, yaw, and xyz translation. This matches the real robot's unconstrained motion.
- `gear` on the motor actuator is the single torque scaling knob. Get this right: measure stall torque from SimpleFOC and match it.
- Wheel `friction="1.0 0.1 0.1"`: first value dominates for rolling (sliding friction), second is torsional, third is rolling resistance.
- Verify the model visually before any training: `python -c "import mujoco, mujoco.viewer; m = mujoco.MjModel.from_xml_path('src/two_wheel_mjlab/robot/two_wheel/two_wheel.xml'); mujoco.viewer.launch(m, mujoco.MjData(m))"`

---

## 4. Step 3: Environment Configuration

File: `src/two_wheel_mjlab/tasks/balance_env_cfg.py`

mjlab uses Isaac Lab's **manager-based declarative pattern**: instead of writing `step()` and `reset()` by hand, you define what the observations, rewards, terminations, and randomization events *are*, and the framework assembles the env loop and vectorizes it across GPU environments.

The exact built-in term names available in mjlab are still evolving (it's beta). Check mjlab_upkie's `balance_env_cfg.py` as the canonical reference — copy it and adapt rather than starting from scratch. The structure below is representative:

```python
from dataclasses import dataclass, field
from mjlab import ManagerBasedRLEnvCfg
from mjlab.managers import (
    ObservationGroupCfg, RewardsCfg, TerminationsCfg, EventCfg
)
import mjlab.managers.observation_terms as obs_terms
import mjlab.managers.reward_terms as rew_terms
import mjlab.managers.event_terms as evt_terms
import mjlab.managers.termination_terms as term_terms
from mjlab.scene import InteractiveSceneCfg
from mjlab.assets import ArticulationCfg


# ── Scene ──────────────────────────────────────────────────────────────────────

@dataclass
class TwoWheelSceneCfg(InteractiveSceneCfg):
    num_envs: int = 2048
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=ArticulationCfg.SpawnCfg(
            usd_path="robot/two_wheel/two_wheel.xml"
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.065)),
    )


# ── Observations (9-dim, matching Arduino sensor availability) ─────────────────
#   [pitch, pitch_rate, roll, roll_rate, left_wheel_vel, right_wheel_vel,
#    x_vel, y_vel, yaw_rate]

@dataclass
class PolicyObsCfg(ObservationGroupCfg):
    pitch:       ... = obs_terms.base_euler_xyz(indices=[1])       # rad
    pitch_rate:  ... = obs_terms.base_ang_vel(indices=[1])         # rad/s
    roll:        ... = obs_terms.base_euler_xyz(indices=[0])       # rad
    roll_rate:   ... = obs_terms.base_ang_vel(indices=[0])         # rad/s
    left_vel:    ... = obs_terms.joint_vel(joint_names=["left_wheel_joint"])
    right_vel:   ... = obs_terms.joint_vel(joint_names=["right_wheel_joint"])
    base_lin_vel:... = obs_terms.base_lin_vel(indices=[0, 1])      # x, y only
    yaw_rate:    ... = obs_terms.base_ang_vel(indices=[2])         # rad/s

    # Observation noise (sigma, applied at policy input — not to recorded state)
    noise_cfg: ... = obs_terms.GaussianNoiseCfg(
        pitch=0.01, pitch_rate=0.05, roll=0.01, roll_rate=0.05
    )


@dataclass
class TwoWheelObsCfg:
    policy: PolicyObsCfg = field(default_factory=PolicyObsCfg)


# ── Rewards ────────────────────────────────────────────────────────────────────

@dataclass
class TwoWheelRewardsCfg(RewardsCfg):
    # Stay upright: 1.0 - 2*pitch^2  (max ~1.0 when pitch=0)
    upright:      ... = rew_terms.upright_posture(weight=1.0, axis="pitch")
    # Don't drift laterally
    no_drift:     ... = rew_terms.base_lin_vel_penalty(weight=0.1, indices=[0, 1])
    # Smooth torque commands
    smooth_act:   ... = rew_terms.action_rate_penalty(weight=0.05)
    # Don't thrash in pitch
    no_tilt_vel:  ... = rew_terms.base_ang_vel_penalty(weight=0.1, indices=[1])


# ── Terminations ───────────────────────────────────────────────────────────────

@dataclass
class TwoWheelTermsCfg(TerminationsCfg):
    # Fallen: |pitch| > 0.5 rad (~28 deg)
    fallen:  ... = term_terms.bad_orientation(limit_angle=0.5, axis="pitch")
    timeout: ... = term_terms.time_out(max_episode_length_s=10.0)


# ── Domain Randomization Events (applied at reset) ────────────────────────────

@dataclass
class TwoWheelEventCfg(EventCfg):
    rand_friction: ... = evt_terms.randomize_rigid_body_material(
        asset_cfg=..., static_friction_range=(0.48, 1.12),  # ±40%
    )
    rand_mass:     ... = evt_terms.randomize_rigid_body_mass(
        asset_cfg=..., mass_distribution_params=(0.8, 1.2),
    )
    rand_damping:  ... = evt_terms.randomize_joint_parameters(
        asset_cfg=..., damping_distribution_params=(0.7, 1.3),
    )
    rand_motor:    ... = evt_terms.randomize_actuator_gains(
        asset_cfg=..., stiffness_distribution_params=(0.8, 1.2),
    )
    init_tilt:     ... = evt_terms.reset_root_state_uniform(
        pose_range={"pitch": (-0.08, 0.08)},  # ±5 deg initial tilt
    )


# ── Top-level config ───────────────────────────────────────────────────────────

@dataclass
class TwoWheelBalanceCfg(ManagerBasedRLEnvCfg):
    scene:        TwoWheelSceneCfg    = field(default_factory=TwoWheelSceneCfg)
    observations: TwoWheelObsCfg     = field(default_factory=TwoWheelObsCfg)
    rewards:      TwoWheelRewardsCfg  = field(default_factory=TwoWheelRewardsCfg)
    terminations: TwoWheelTermsCfg   = field(default_factory=TwoWheelTermsCfg)
    events:       TwoWheelEventCfg   = field(default_factory=TwoWheelEventCfg)

    def __post_init__(self):
        self.sim.dt = 0.002           # 2ms physics timestep
        self.decimation = 5           # policy runs at 100 Hz (every 5 physics steps)
        super().__post_init__()
```

Register in `src/two_wheel_mjlab/__init__.py`:

```python
import mjlab
from .tasks.balance_env_cfg import TwoWheelBalanceCfg

mjlab.register(
    id="TwoWheel-Balance-v0",
    entry_point=TwoWheelBalanceCfg,
)
```

> **API note:** Term names like `obs_terms.base_euler_xyz`, `rew_terms.upright_posture`, `evt_terms.randomize_rigid_body_material` are patterned after Isaac Lab's built-in term library. mjlab's exact term names differ — consult `mjlab/managers/` source and mjlab_upkie's config as the ground truth. Many Isaac Lab terms are directly usable; custom terms are plain Python functions over `env.scene.robot` data.

---

## 5. Step 4: Train a Baseline Policy

mjlab uses **RSL-RL** (on-policy PPO) as its training backend. Training is launched via the `train` entry point installed by mjlab.

```bash
# Baseline: no domain randomization, small env count to verify learning
uv run train TwoWheel-Balance-v0 \
    --env.scene.num-envs 256 \
    --max-iterations 500

# Full training run once baseline works
uv run train TwoWheel-Balance-v0 \
    --env.scene.num-envs 2048 \
    --max-iterations 2000
```

Checkpoints land in `logs/rsl_rl/TwoWheel-Balance-v0/<timestamp>/`. W&B logging is on by default; disable with `--no-wandb` if needed.

### RSL-RL PPO defaults (likely to work well, adjust if needed)

| Hyperparameter | Default | Notes |
|---|---|---|
| `num_steps_per_env` | 24 | Rollout length per env per update |
| `num_mini_batches` | 4 | Mini-batch count per epoch |
| `num_learning_epochs` | 5 | PPO epochs per update |
| `learning_rate` | 1e-3 | Reduce to 3e-4 if unstable |
| `entropy_coef` | 0.01 | Increase if policy collapses early |
| `clip_param` | 0.2 | Standard PPO clip |

### Network architecture

RSL-RL's default actor-critic for locomotion tasks:

```
Actor:  obs(9) → Linear(128) → ELU → Linear(64) → ELU → Linear(64) → ELU → Linear(2)
Critic: obs(9) → Linear(128) → ELU → Linear(64) → ELU → Linear(64) → ELU → Linear(1)
```

This is larger than needed for deployment but trains reliably. For Arduino export, you can reduce to 64→64 by overriding the `rsl_rl_cfg` in the env config — validate that smaller architecture still converges before committing.

RSL-RL applies **running obs normalization** (`EmpiricalNormalization`) by default. The running mean and std must be exported alongside the weights (see Step 7).

### Convergence signal

A well-tuned policy should show:
- Episode length consistently reaching `max_episode_length_s` (10s) within ~500 iterations at 2048 envs
- Mean episodic return rising and plateauing
- Pitch staying within ±0.1 rad during steady-state in `sim.py`

---

## 6. Step 5: Evaluate and Visualize (CPU)

mjlab_upkie ships a `sim.py` that loads a trained RSL-RL checkpoint and runs it against a standard `mujoco.viewer` on CPU. Copy and adapt it:

```bash
python sim.py \
    --task TwoWheel-Balance-v0 \
    --checkpoint logs/rsl_rl/TwoWheel-Balance-v0/<timestamp>/model_<iter>.pt
```

Use this to:
- Visually confirm the policy is balancing and not doing something pathological
- Apply manual push perturbations (mjlab_upkie supports force injection in sim.py) to test robustness
- Compare obs distributions between sim and real hardware logs (log real IMU data during PID operation and overlay)

---

## 7. Step 6: Domain Randomization

Randomization is declared in `TwoWheelEventCfg` (Step 3). **Start without randomization** (comment out all event terms), confirm the policy balances, then add randomization one group at a time.

### Parameter ranges

| Parameter | mjlab EventCfg term | Range | Rationale |
|---|---|---|---|
| Wheel/floor friction | `randomize_rigid_body_material` | ±40% | Surface variation |
| Body masses | `randomize_rigid_body_mass` | ±20% per body | Battery depletion shifts CoM |
| Joint damping | `randomize_joint_parameters` | ±30% | BLDC cogging, back-EMF variation |
| Motor torque gain | `randomize_actuator_gains` | ±20% | SimpleFOC current loop calibration error |
| IMU obs noise | `GaussianNoiseCfg` in obs | σ = 0.01 rad angle, 0.05 rad/s rate | nRF52840 IMU noise floor |
| CoM offset | `randomize_rigid_body_mass` with ipos | ±5mm | Uneven weight placement |
| **Action latency** | Custom delay term | 1–3 steps (10–30ms) | BLE + compute delay |

### Action latency (most impactful for real hardware)

mjlab doesn't have a built-in latency term yet. Implement as a custom observation/event term that maintains a ring buffer of past actions and feeds a delayed action to the actuator. See Isaac Lab's action delay wrapper as a reference pattern. Add this last — it's the hardest to train through.

### Curriculum schedule

mjlab's `EventCfg` supports probability and interval parameters. Implement a curriculum by starting with narrow randomization ranges and widening them as a function of training iteration:

```python
# Override event cfg at training start via curriculum callback
# (exact mjlab API for curriculum — check current docs/mjlab_upkie)
rand_mass = evt_terms.randomize_rigid_body_mass(
    asset_cfg=...,
    mass_distribution_params=(0.95, 1.05),  # start narrow: ±5%
)
# Widen to (0.8, 1.2) after policy is stable
```

---

## 8. Step 7: Export Weights and Deploy to Arduino

The RSL-RL actor is a standard `torch.nn.Sequential` MLP. Export it (and the obs normalizer stats) to C header files.

### Hardware budget (nRF52840: 1MB Flash, 256KB RAM, Cortex-M4 @ 64MHz)

| Network | Parameters | Flash (float32) | Inference RAM |
|---|---|---|---|
| 128→64→64→2 (RSL-RL default) | ~18k | ~72 KB | ~2 KB |
| 64→64→2 (reduced) | ~4.9k | ~20 KB | ~512 B |

Both fit comfortably. Prefer the smaller network if it converges equally well.

### Export script (`export_weights.py`)

```python
import torch
import numpy as np

def to_c_array(tensor, name):
    arr = tensor.detach().cpu().numpy()
    shape = "][".join(str(d) for d in arr.shape)
    vals = ", ".join(f"{v:.8f}f" for v in arr.flatten())
    return f"static const float {name}[{shape}] = {{{vals}}};\n"

checkpoint_path = "logs/rsl_rl/TwoWheel-Balance-v0/.../model_final.pt"
ckpt = torch.load(checkpoint_path, map_location="cpu")

# RSL-RL checkpoint layout:
# ckpt["model_state_dict"] contains actor_body, std, obs_normalizer
state = ckpt["model_state_dict"]

# Extract actor linear layers (skip activation modules)
actor_layers = [
    (state[f"actor.{k}"], state[f"actor.{b}"])
    for k, b in [
        ("0.weight", "0.bias"),   # adjust indices to match your arch
        ("2.weight", "2.bias"),
        ("4.weight", "4.bias"),
    ]
]

# Export obs normalizer running stats
obs_mean = state["normalizer.running_mean"].numpy()
obs_var  = state["normalizer.running_var"].numpy()
obs_std  = np.sqrt(obs_var + 1e-8)

with open("firmware/policy_weights.h", "w") as f:
    f.write("#pragma once\n\n")
    for i, (W, b) in enumerate(actor_layers, start=1):
        f.write(to_c_array(W, f"W{i}"))
        f.write(to_c_array(b, f"b{i}"))

with open("firmware/obs_norm.h", "w") as f:
    f.write("#pragma once\n\n")
    f.write(to_c_array(torch.tensor(obs_mean), "OBS_MEAN"))
    f.write(to_c_array(torch.tensor(obs_std),  "OBS_STD"))

print("Done.")
```

> RSL-RL's exact state dict key names depend on its version and your network config. Print `state.keys()` to verify before assuming.

### C++ inference (`firmware/policy.h`)

Assumes ELU activation (RSL-RL default). If you configured Tanh, replace `elu()` with `tanhf()`.

```cpp
#pragma once
#include <math.h>
#include "policy_weights.h"
#include "obs_norm.h"

// ELU activation
static inline float elu(float x) { return x >= 0.0f ? x : (expf(x) - 1.0f); }

// Network: 9 -> 64 -> 64 -> 2 (adjust sizes if using 128->64->64)
#define OBS_DIM 9
#define H1 64
#define H2 64
#define ACT_DIM 2

void policy_forward(const float raw_obs[OBS_DIM], float action[ACT_DIM]) {
    float obs[OBS_DIM], h1[H1], h2[H2];

    // Normalize observations using exported running stats
    for (int i = 0; i < OBS_DIM; i++)
        obs[i] = (raw_obs[i] - OBS_MEAN[i]) / OBS_STD[i];

    // Layer 1
    for (int i = 0; i < H1; i++) {
        float s = b1[i];
        for (int j = 0; j < OBS_DIM; j++) s += W1[i][j] * obs[j];
        h1[i] = elu(s);
    }

    // Layer 2
    for (int i = 0; i < H2; i++) {
        float s = b2[i];
        for (int j = 0; j < H1; j++) s += W2[i][j] * h1[j];
        h2[i] = elu(s);
    }

    // Output (no activation — RSL-RL clips actions externally)
    for (int i = 0; i < ACT_DIM; i++) {
        float s = b3[i];
        for (int j = 0; j < H2; j++) s += W3[i][j] * h2[j];
        action[i] = fmaxf(-1.0f, fminf(1.0f, s));
    }
}
```

### Arduino sketch (`firmware/policy_inference.ino`)

```cpp
#include "policy.h"
// ... SimpleFOC and IMU setup (LSM9DS1 on Nano 33 BLE) ...

#define MAX_TORQUE_NM 0.15f  // tune: measured SimpleFOC peak torque

void loop() {
    float obs[OBS_DIM];

    // Populate from hardware — sign conventions must match MJCF
    obs[0] = imu.getPitch();               // rad (positive = forward lean)
    obs[1] = imu.getPitchRate();           // rad/s
    obs[2] = imu.getRoll();                // rad
    obs[3] = imu.getRollRate();            // rad/s
    obs[4] = motor_left.shaft_velocity;    // rad/s
    obs[5] = motor_right.shaft_velocity;   // rad/s
    obs[6] = (obs[4] + obs[5]) * 0.5f * WHEEL_RADIUS;  // approx x_vel
    obs[7] = 0.0f;                         // y_vel not measurable
    obs[8] = imu.getYawRate();             // rad/s

    float action[ACT_DIM];
    policy_forward(obs, action);

    motor_left.move(action[0]  * MAX_TORQUE_NM);
    motor_right.move(action[1] * MAX_TORQUE_NM);

    delayMicroseconds(10000);  // 100 Hz
}
```

### Validate before flashing

Write a Python script that runs both the PyTorch model and the C inference (compiled via `cffi` or as a standalone C program) on the same input and asserts agreement to within 1e-4. This catches weight layout bugs before you're debugging on hardware.

---

## 9. Key Gotchas

| Issue | Solution |
|---|---|
| mjlab API changed since this doc was written | mjlab is beta — always check current source and mjlab_upkie for ground truth |
| Policy doesn't transfer to real robot | Check: (1) obs normalization stats exported correctly, (2) action scaling, (3) sign conventions for pitch/torque, (4) latency not modeled in sim |
| IMU pitch sign convention mismatch | Log raw IMU values while running PID controller on real robot; verify sign and magnitude match MJCF conventions before enabling RL |
| RSL-RL state dict key names don't match export script | Print `state.keys()` from the checkpoint; rsl-rl changes key names across versions |
| Training unstable with domain randomization | Add one parameter group at a time; start with ±10% and widen; action latency is usually the hardest — add last |
| Policy exploits simulation artifacts | Add more randomization; randomize gravity slightly (±5%); check that floor contact model is reasonable |
| `MAX_TORQUE_NM` wrong | Measure: command a known current in SimpleFOC, measure output shaft torque with a scale. This scalar directly determines whether the policy's [-1,1] output maps to sensible torques. |
| ELU vs Tanh activation mismatch | RSL-RL default is ELU. If you configured Tanh in training, use `tanhf()` in `policy.h`. Verify by comparing Python and C outputs on a test input. |

---

## 10. Sources

- [mjlab GitHub](https://github.com/mujocolab/mjlab)
- [mjlab_upkie: wheeled biped RL on mjlab (primary reference)](https://github.com/MarcDcls/mjlab_upkie)
- [RSL-RL (PPO training backend)](https://github.com/leggedrobotics/rsl_rl)
- [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Modeling Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [Isaac Lab Manager-Based Env Docs](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/create_manager_rl_env.html) (same API pattern as mjlab)
- [Isaac Lab Built-in Observation/Reward/Event Terms](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.mdp.html) (reference for what terms exist; mjlab has a subset)
- [LiteRT (TFLite) for Microcontrollers](https://ai.google.dev/edge/litert/microcontrollers/overview) (alternative to manual C++ inference if needed)
- [MuJoCo Menagerie (reference robot models)](https://github.com/google-deepmind/mujoco_menagerie)
