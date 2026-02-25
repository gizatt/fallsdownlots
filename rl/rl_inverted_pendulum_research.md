# RL Controller for Two-Wheeled Inverted Pendulum: Research & Implementation Plan

**Robot:** Altoids tin chassis, two BLDC outrunners, SimpleFOC, magnetic encoder, Arduino Nano 33 BLE (nRF52840), onboard IMU.
**Goal:** Train an RL policy in MuJoCo simulation and deploy it to the Arduino.

---

## Table of Contents

1. [MuJoCo Ecosystem Overview](#1-mujoco-ecosystem-overview)
2. [Framework Comparison: Standalone vs mjlab](#2-framework-comparison-standalone-vs-mjlab)
3. [Recommended Project Structure (Standalone)](#3-recommended-project-structure-standalone)
4. [mjlab-Based Project Structure](#4-mjlab-based-project-structure)
5. [Phase 1: MuJoCo Environment](#5-phase-1-mujoco-environment)
6. [Phase 2: RL Algorithm Choice](#6-phase-2-rl-algorithm-choice)
7. [Phase 3: Training Infrastructure](#7-phase-3-training-infrastructure)
8. [Phase 4: Domain Randomization](#8-phase-4-domain-randomization)
9. [Phase 5: Deploying to Arduino Nano 33 BLE](#9-phase-5-deploying-to-arduino-nano-33-ble)
10. [Implementation Sequence](#10-implementation-sequence)
11. [Sources](#11-sources)

---

## 1. MuJoCo Ecosystem Overview

### MuJoCo Ecosystem Map

| Package | Purpose | Backend | GPU? | Use When |
|---|---|---|---|---|
| `mujoco` | Standard Python bindings | CPU (C) | No | Quick prototyping, CPU-only |
| `mujoco-mjx` | JAX-accelerated MuJoCo | JAX (GPU/TPU) | Yes | Scaling to 1000s of envs (JAX) |
| `mujoco_playground` | Full RL framework on MJX | JAX | Yes | Full locomotion suite (JAX) |
| `mjlab` | Isaac Lab API on MuJoCo Warp | CUDA/Warp | Yes (NVIDIA) | Isaac Lab-style, NVIDIA GPU training |
| `mujoco-py` | Old OpenAI bindings | CPU | No | Deprecated — avoid |

`mjlab` ([github.com/mujocolab/mjlab](https://github.com/mujocolab/mjlab)) adopts Isaac Lab's manager-based composable API and pairs it with MuJoCo Warp (NVIDIA GPU physics). It is **directly analogous** to the Upkie wheeled biped project [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie), which trains a velocity-tracking and balancing policy for a wheeled biped robot — making it a very close reference for this project.

---

## 2. Framework Comparison: Standalone vs mjlab

### High-Level Tradeoffs

| Dimension | Standalone (`mujoco` + CleanRL) | mjlab |
|---|---|---|
| **GPU requirement** | None (CPU training works fine) | **NVIDIA GPU required** for training |
| **Parallelism** | 8–32 CPU envs (VectorEnv) | 256–4096+ GPU envs (MuJoCo Warp) |
| **Wall-clock training** | ~30–60 min (PPO, 8 envs, 3M steps) | ~5–15 min (2048 envs, same budget) |
| **Algorithm** | PPO, SAC, TD3 (full CleanRL library) | RSL-RL PPO (main); SAC not native |
| **Environment definition** | Python class (`gym.Env`), explicit `step()`/`reset()` | Declarative manager config (`env_cfg.py`), no explicit step |
| **Domain randomization** | Manual code in `_apply_domain_randomization()` | `EventManager` terms, declarative |
| **Logging** | TensorBoard (+ optional W&B) | W&B (primary), TensorBoard possible |
| **Learning curve** | Moderate — gymnasium + CleanRL are well-documented | Steeper — Isaac Lab paradigm (managers, `@configclass`) |
| **Stability** | Stable, mature dependencies | Beta — breaking changes expected |
| **Prior art for this robot** | Generic continuous-control examples | [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie): wheeled biped, nearly identical task |
| **Weight export to Arduino** | Straightforward (PyTorch → C arrays) | Same (RSL-RL actor is a PyTorch MLP) |

### When to Choose mjlab

Choose mjlab if:
- You have an **NVIDIA GPU** available for training
- You want **10–100x faster iteration** via massive parallelism (critical for domain randomization sweeps)
- You are willing to learn the Isaac Lab manager API (it scales well to more complex tasks)
- You want a ready-made reference: `mjlab_upkie` is almost exactly this project (wheeled biped, velocity control, sim-to-real)

Choose standalone if:
- You are training on **CPU only** (no NVIDIA GPU)
- You want the fastest possible **time-to-first-result** with minimal boilerplate
- You prefer SAC (better sample efficiency on CPU) — mjlab/RSL-RL is PPO-focused
- You want maximum flexibility over reward shaping and observation construction

### Decision Summary

> **If you have an NVIDIA GPU, mjlab is the stronger choice.** The mjlab_upkie reference implementation is a wheeled biped doing exactly what this project needs. Starting from it avoids writing a Gymnasium env from scratch and gets you to 2048 parallel environments immediately. The Isaac Lab config pattern is more verbose but scales better as the task complexity grows.
>
> **If training on CPU, use the standalone approach.** The gymnasium + CleanRL path is simpler, SAC is available, and 8 CPU envs is enough to train a basic balancer in under an hour.

---

## 3. Recommended Project Structure (Standalone)

```
rl/
├── envs/
│   ├── __init__.py              # gym.register() call
│   ├── two_wheel_env.py         # Gymnasium environment
│   └── two_wheel_robot.xml      # MJCF model
├── train_ppo.py                 # Single-file PPO training
├── train_sac.py                 # SAC alternative (copy from CleanRL)
├── export_weights.py            # PyTorch -> C array exporter
├── checkpoints/                 # Saved .pt checkpoints
├── runs/                        # TensorBoard logs
├── videos/                      # Periodic rendered episodes
└── firmware/
    ├── policy_weights.h         # Generated C weight arrays
    └── policy_inference.ino     # Arduino sketch
```

Install dependencies:
```bash
pip install "gymnasium[mujoco]" mujoco torch tensorboard cleanrl
```

---

## 4. mjlab-Based Project Structure

Based directly on [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie), adapted for a two-wheeled balancer.

```
rl/
├── src/
│   └── two_wheel_mjlab/
│       ├── __init__.py
│       ├── tasks/
│       │   └── balance_env_cfg.py       # Isaac Lab-style manager config
│       └── robot/
│           └── two_wheel/
│               └── two_wheel.xml        # MJCF model (same as standalone)
├── sim.py                               # CPU-side playback via mujoco.viewer
├── export_weights.py                    # RSL-RL checkpoint -> C array
├── pyproject.toml                       # uv/pip metadata
├── logs/                                # RSL-RL checkpoints + W&B artifacts
└── firmware/
    ├── policy_weights.h                 # Generated C weight arrays
    └── policy_inference.ino             # Arduino sketch (same as standalone)
```

Install:
```bash
pip install mjlab rsl-rl wandb
# or: uv add mjlab rsl-rl wandb
```

### 4.1 Environment Configuration (`tasks/balance_env_cfg.py`)

Rather than implementing `step()` and `reset()` by hand, mjlab uses a **declarative manager pattern** — you define *what* the observations, rewards, terminations, and randomization events are; the framework assembles the env loop:

```python
from dataclasses import dataclass, field
import mjlab
from mjlab import ManagerBasedRLEnvCfg, SceneEntityCfg
from mjlab.managers import ObservationsCfg, RewardsCfg, TerminationsCfg, EventCfg, CommandsCfg
import mjlab.managers.observation_terms as obs_terms
import mjlab.managers.reward_terms as rew_terms
import mjlab.managers.event_terms as evt_terms
import mjlab.managers.termination_terms as term_terms


@dataclass
class TwoWheelSceneCfg(SceneEntityCfg):
    num_envs: int = 2048
    dt: float = 0.002
    substeps: int = 5          # 10ms per policy step (100 Hz)
    robot_usd: str = "two_wheel/two_wheel.xml"


@dataclass
class TwoWheelObsCfg(ObservationsCfg):
    # These map to mjlab built-in terms or custom lambdas
    pitch:       obs_terms.ImuRpy      = field(default_factory=lambda: obs_terms.ImuRpy(indices=[1]))
    pitch_rate:  obs_terms.ImuGyro     = field(default_factory=lambda: obs_terms.ImuGyro(indices=[1]))
    roll:        obs_terms.ImuRpy      = field(default_factory=lambda: obs_terms.ImuRpy(indices=[0]))
    roll_rate:   obs_terms.ImuGyro     = field(default_factory=lambda: obs_terms.ImuGyro(indices=[0]))
    left_wheel:  obs_terms.JointVel    = field(default_factory=lambda: obs_terms.JointVel(joint="left_wheel_joint"))
    right_wheel: obs_terms.JointVel    = field(default_factory=lambda: obs_terms.JointVel(joint="right_wheel_joint"))
    base_vel:    obs_terms.BaseLinVel  = field(default_factory=obs_terms.BaseLinVel)
    yaw_rate:    obs_terms.BaseAngVel  = field(default_factory=lambda: obs_terms.BaseAngVel(indices=[2]))


@dataclass
class TwoWheelRewardsCfg(RewardsCfg):
    # Weighted reward terms; mjlab sums them each step
    upright:     rew_terms.UprightPitch     = field(default_factory=lambda: rew_terms.UprightPitch(weight=1.0))
    no_drift:    rew_terms.BaseLinVelPenalty = field(default_factory=lambda: rew_terms.BaseLinVelPenalty(weight=0.1))
    smooth_act:  rew_terms.ActionRatePenalty = field(default_factory=lambda: rew_terms.ActionRatePenalty(weight=0.05))
    no_tilt_vel: rew_terms.PitchRatePenalty  = field(default_factory=lambda: rew_terms.PitchRatePenalty(weight=0.1))


@dataclass
class TwoWheelTerminationsCfg(TerminationsCfg):
    fallen: term_terms.BadOrientation = field(
        default_factory=lambda: term_terms.BadOrientation(limit_pitch=0.5)  # ~28 deg
    )
    timeout: term_terms.TimeOut = field(default_factory=lambda: term_terms.TimeOut(max_episode_steps=1000))


@dataclass
class TwoWheelEventCfg(EventCfg):
    """Domain randomization applied at reset."""
    rand_friction:  evt_terms.RandomizeFriction    = field(default_factory=lambda: evt_terms.RandomizeFriction(scale=(0.6, 1.4)))
    rand_mass:      evt_terms.RandomizeMass        = field(default_factory=lambda: evt_terms.RandomizeMass(scale=(0.8, 1.2)))
    rand_damping:   evt_terms.RandomizeDamping     = field(default_factory=lambda: evt_terms.RandomizeDamping(scale=(0.7, 1.3)))
    rand_motor:     evt_terms.RandomizeActuatorGain = field(default_factory=lambda: evt_terms.RandomizeActuatorGain(scale=(0.8, 1.2)))
    init_tilt:      evt_terms.RandomizeInitPitch   = field(default_factory=lambda: evt_terms.RandomizeInitPitch(range=(-0.08, 0.08)))


@dataclass
class TwoWheelBalanceCfg(ManagerBasedRLEnvCfg):
    scene:        TwoWheelSceneCfg       = field(default_factory=TwoWheelSceneCfg)
    observations: TwoWheelObsCfg         = field(default_factory=TwoWheelObsCfg)
    rewards:      TwoWheelRewardsCfg     = field(default_factory=TwoWheelRewardsCfg)
    terminations: TwoWheelTerminationsCfg = field(default_factory=TwoWheelTerminationsCfg)
    events:       TwoWheelEventCfg       = field(default_factory=TwoWheelEventCfg)
```

> **Note:** The exact manager term names (`obs_terms.ImuRpy`, `rew_terms.UprightPitch`, etc.) depend on mjlab's built-in library, which is still evolving. You may need to define custom terms as lambdas over MuJoCo data. The pattern is identical to Isaac Lab — see mjlab_upkie's `balance_env_cfg.py` for a concrete reference.

### 4.2 Register and Train

```python
# __init__.py
import mjlab
mjlab.register(
    id="TwoWheel-Balance-v0",
    entry_point=TwoWheelBalanceCfg,
)
```

```bash
# Train (NVIDIA GPU required):
uv run train TwoWheel-Balance-v0 --env.scene.num-envs 2048

# Play / evaluate on CPU:
python sim.py --checkpoint logs/rsl_rl/two_wheel_balance/bests/model.pt
```

Training uses **RSL-RL** (on-policy PPO) with automatic W&B logging.

### 4.3 CPU Fallback for Evaluation (`sim.py`)

mjlab_upkie ships a `sim.py` that runs the trained policy against a standard `mujoco.viewer` on CPU — identical to the standalone approach. The weight export and Arduino firmware are also identical since both produce a plain PyTorch MLP.

### 4.4 Key Differences Calling Out

| Topic | Standalone | mjlab |
|---|---|---|
| **Env loop** | Explicit Python (`step`, `reset`, `_get_obs`) | Implicit — framework assembles from manager terms |
| **Domain randomization** | Manual code in `_apply_domain_randomization()` | Declarative `EventCfg` terms at `reset` |
| **Reward shaping** | Python arithmetic in `_compute_reward()` | Weighted `RewardsCfg` terms summed by framework |
| **Observation noise** | Explicit numpy in `_get_obs()` | Noise added as an `ObservationNoiseCfg` term |
| **Scaling** | `num_envs=8` (CPU VectorEnv) | `num_envs=2048` (GPU, ~100x faster) |
| **Algorithm access** | Full CleanRL (PPO, SAC, TD3) | RSL-RL PPO (SAC not natively supported) |
| **Starting point** | From scratch | Fork [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie) |

### 4.5 mjlab Weight Export (Same as Standalone)

RSL-RL's actor is a standard `torch.nn.Sequential` MLP, so the export script from Section 9 (Option A) works without modification — just point it at the RSL-RL checkpoint instead of a CleanRL checkpoint.

---

## 5. Phase 1: MuJoCo Environment

### 3.1 MJCF Robot Model (`envs/two_wheel_robot.xml`)

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

    <!-- Chassis: the "pendulum" body -->
    <body name="chassis" pos="0 0 0.065">
      <!-- freejoint allows unconstrained 3D motion -->
      <!-- For strict 2D: replace with slide(x) + slide(z) + hinge(pitch-Y) -->
      <freejoint name="chassis_free"/>
      <!-- ~80g representing altoids tin + battery + electronics -->
      <geom name="chassis_box" type="box" size="0.03 0.025 0.05"
            mass="0.08" rgba="0.6 0.3 0.1 1"/>
      <site name="imu_site" pos="0 0 0" size="0.005"/>

      <body name="left_wheel" pos="0 -0.04 -0.05">
        <!-- Hinge spins around Y axis -->
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
    <!-- gear scales torque; tune to match measured SimpleFOC stall torque (~0.1-0.3 Nm) -->
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

**Key modeling decisions:**
- `freejoint` gives full 6-DOF chassis motion (pitch, roll, yaw, xyz translation). This matches the real robot.
- `gear` on the motor actuator is the torque scaling constant. Tune this by measuring max torque output from SimpleFOC.
- Wheel `friction="1.0 0.1 0.1"`: first value is sliding friction (dominant for rolling), second torsional, third rolling resistance.

### 3.2 Gymnasium Environment (`envs/two_wheel_env.py`)

```python
"""
Gymnasium-compatible two-wheeled balancing robot environment.
Wraps MuJoCo via the standard `mujoco` Python bindings.
"""
import os
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces


class TwoWheelBalancerEnv(gym.Env):
    """
    Observation space (9-dim):
        [0] chassis pitch angle (rad)
        [1] chassis pitch rate (rad/s)
        [2] chassis roll angle (rad)
        [3] chassis roll rate (rad/s)
        [4] left wheel velocity (rad/s)
        [5] right wheel velocity (rad/s)
        [6] x velocity (m/s)
        [7] y velocity (m/s)
        [8] yaw rate (rad/s)

    Action space (2-dim):
        [0] left motor torque  in [-1, 1]
        [1] right motor torque in [-1, 1]
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, xml_path=None, domain_randomize=True):
        super().__init__()

        if xml_path is None:
            xml_path = os.path.join(os.path.dirname(__file__), "two_wheel_robot.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self.domain_randomize = domain_randomize

        # Store nominal model parameters for domain randomization
        self._nominal_geom_friction  = self.model.geom_friction.copy()
        self._nominal_body_mass      = self.model.body_mass.copy()
        self._nominal_dof_damping    = self.model.dof_damping.copy()
        self._nominal_actuator_gear  = self.model.actuator_gear[:, 0].copy()

        self.dt           = self.model.opt.timestep  # 0.002 s
        self.frame_skip   = 5                         # -> 10ms per env step (100 Hz)
        self.max_episode_steps = 1000                 # 10 seconds

        obs_high = np.array([
            np.pi, 20.0, np.pi, 20.0,   # pitch, pitch_rate, roll, roll_rate
            50.0, 50.0,                  # wheel velocities
            3.0, 3.0, 10.0,             # x_vel, y_vel, yaw_rate
        ], dtype=np.float32)

        self.observation_space = spaces.Box(low=-obs_high, high=obs_high, dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self._step_count = 0
        self._renderer = None
        self.render_mode = render_mode

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        if self.domain_randomize:
            self._apply_domain_randomization()

        # Small random initial tilt: ±5 degrees
        pitch_init = self.np_random.uniform(-0.08, 0.08)
        # chassis_free joint: qpos[0:3]=xyz, qpos[3:7]=quaternion [w,x,y,z]
        half = pitch_init / 2.0
        self.data.qpos[3] = np.cos(half)  # w
        self.data.qpos[4] = 0.0            # x
        self.data.qpos[5] = np.sin(half)   # y (pitch axis)
        self.data.qpos[6] = 0.0            # z

        self._step_count = 0
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0).astype(np.float64)
        self.data.ctrl[:] = action

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        reward = self._compute_reward(obs, action)

        pitch = obs[0]
        terminated = bool(abs(pitch) > 0.5)  # ~28 degrees -> fallen
        self._step_count += 1
        truncated = self._step_count >= self.max_episode_steps

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, {}

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        self._renderer.update_scene(self.data)
        if self.render_mode == "rgb_array":
            return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_obs(self):
        # chassis_free joint: qpos[0:3]=xyz, qpos[3:7]=quat(w,x,y,z)
        #                     qvel[0:3]=xyz_vel, qvel[3:6]=angular_vel
        quat = self.data.qpos[3:7]
        pitch, roll = _quat_to_pitch_roll(quat)

        pitch_rate = self.data.qvel[4]  # angular vel around Y (pitch axis)
        roll_rate  = self.data.qvel[3]  # angular vel around X (roll axis)
        yaw_rate   = self.data.qvel[5]  # angular vel around Z (yaw axis)
        x_vel      = self.data.qvel[0]
        y_vel      = self.data.qvel[1]

        # Wheel joints follow freejoint: qvel indices 6 and 7
        lw_vel = self.data.qvel[6]
        rw_vel = self.data.qvel[7]

        obs = np.array([
            pitch, pitch_rate, roll, roll_rate,
            lw_vel, rw_vel,
            x_vel, y_vel, yaw_rate,
        ], dtype=np.float32)

        # Add IMU-like sensor noise
        obs[0] += self.np_random.normal(0, 0.01)   # pitch noise  (~0.5 deg)
        obs[1] += self.np_random.normal(0, 0.05)   # pitch rate noise
        obs[2] += self.np_random.normal(0, 0.01)   # roll noise
        obs[3] += self.np_random.normal(0, 0.05)   # roll rate noise

        return obs

    def _compute_reward(self, obs, action):
        pitch, pitch_rate = obs[0], obs[1]
        x_vel, y_vel      = obs[6], obs[7]

        r_upright  =  1.0 - 2.0 * (pitch**2)       # reward for staying upright
        r_still    = -0.1 * (x_vel**2 + y_vel**2)  # penalize drifting
        r_smooth   = -0.05 * np.sum(action**2)       # penalize torque spikes
        r_tilt_vel = -0.1 * (pitch_rate**2)          # penalize rapid tilting

        return float(r_upright + r_still + r_smooth + r_tilt_vel)

    def _apply_domain_randomization(self):
        rng = self.np_random

        # Friction: ±40%
        self.model.geom_friction[:] = self._nominal_geom_friction * rng.uniform(0.6, 1.4)

        # Body masses: ±20% per body
        self.model.body_mass[:] = (
            self._nominal_body_mass * rng.uniform(0.8, 1.2, size=self._nominal_body_mass.shape)
        )

        # Joint damping: ±30%
        self.model.dof_damping[:] = (
            self._nominal_dof_damping * rng.uniform(0.7, 1.3, size=self._nominal_dof_damping.shape)
        )

        # Motor torque gain: ±20%
        self.model.actuator_gear[:, 0] = self._nominal_actuator_gear * rng.uniform(0.8, 1.2)

        # Recompute derived quantities after changing mass/inertia
        mujoco.mj_setConst(self.model, self.data)


def _quat_to_pitch_roll(q):
    """Extract pitch (around Y) and roll (around X) from quaternion [w, x, y, z]."""
    w, x, y, z = q
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    return pitch, roll
```

Register the environment (`envs/__init__.py`):
```python
import gymnasium as gym
gym.register(
    id="TwoWheelBalancer-v0",
    entry_point="envs.two_wheel_env:TwoWheelBalancerEnv",
    max_episode_steps=1000,
)
```

---

## 6. Phase 2: RL Algorithm Choice

### Comparison Table

| Property | PPO | SAC | TD3 |
|---|---|---|---|
| Type | On-policy | Off-policy | Off-policy |
| Sample efficiency | Low | **High** | Moderate-High |
| Replay buffer | No | Yes (~1M steps) | Yes (~1M steps) |
| Policy type | Stochastic | Stochastic (entropy-max) | Deterministic |
| Exploration | Policy std | Automatic entropy tuning | Manual Gaussian noise |
| Hyperparameter sensitivity | Low | **Low (auto-alpha)** | Moderate |
| Stability | **Very stable** | Very stable | Stable |
| Steps to solve balancing | ~1–5M | ~200–500k | ~500k–1M |
| Beginner-friendly | **Highest** | High | Moderate |

### Recommendation

1. **Start with PPO** — validate that the environment is correct and produces sensible behavior. No replay buffer complexity, very stable.
2. **Switch to SAC** — once PPO works, SAC converges 5–10x faster and handles continuous control very well. The automatic entropy (`log_alpha`) tuning removes manual exploration noise tuning.

**Use [CleanRL](https://github.com/vwxyzjn/cleanrl)** as the base. Their `ppo_continuous_action.py` and `sac_continuous_action.py` are ~400 lines each, single-file, and production-quality. Adapt them to your environment rather than writing from scratch.

### Network Architecture

A small MLP is sufficient and advantageous for deployment:

```
Actor:  obs(9) -> Linear(64) -> Tanh -> Linear(64) -> Tanh -> Linear(2) -> Tanh
Critic: obs(9) -> Linear(64) -> Tanh -> Linear(64) -> Tanh -> Linear(1)
```

Using `Tanh` activations (as in CleanRL's orthogonal-init setup) works well. The 9→64→64→2 actor has only ~4,930 parameters — trivially small for deployment.

---

## 7. Phase 3: Training Infrastructure

### Key Script Features

All features should live in a single training file (modeled on CleanRL):

```python
@dataclasses.dataclass
class Args:
    exp_name:        str   = "two_wheel_ppo"
    seed:            int   = 1
    total_timesteps: int   = 3_000_000
    num_envs:        int   = 8          # parallel environments
    num_steps:       int   = 2048       # rollout length per env
    learning_rate:   float = 3e-4
    # ... PPO hyperparameters ...
    track_wandb:     bool  = False
    save_interval:   int   = 100        # checkpoint every N updates
    render_interval: int   = 200        # record video every N updates
    resume_path: Optional[str] = None
```

### Checkpointing

```python
def save_checkpoint(update, path):
    torch.save({
        "update": update,
        "model_state_dict": agent.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
    }, path)

def load_checkpoint(path):
    ckpt = torch.load(path, map_location=device)
    agent.load_state_dict(ckpt["model_state_dict"])
    optimizer.load_state_dict(ckpt["optimizer_state_dict"])
    return ckpt["update"] + 1  # start_update
```

**Resume usage:**
```bash
python train_ppo.py                                                     # first run
python train_ppo.py --resume_path checkpoints/.../update_100.pt        # resume
```

### TensorBoard Logging

```python
writer = SummaryWriter(f"runs/{run_name}")
# Log per-update:
writer.add_scalar("losses/policy_loss", pg_loss.item(), global_step)
writer.add_scalar("losses/value_loss",  v_loss.item(),  global_step)
writer.add_scalar("losses/entropy",     ent_loss.item(), global_step)
# Log per-episode (from gymnasium RecordEpisodeStatistics wrapper):
writer.add_scalar("charts/episodic_return", ep_ret, global_step)
writer.add_scalar("charts/episodic_length", ep_len, global_step)
```

```bash
tensorboard --logdir runs/
```

### Periodic Policy Visualization

```python
from gymnasium.wrappers import RecordVideo

if update % args.render_interval == 0:
    render_env = gym.make(args.env_id, render_mode="rgb_array")
    render_env = RecordVideo(
        render_env,
        video_folder=f"videos/{run_name}",
        episode_trigger=lambda ep: True,
        name_prefix=f"update_{update}",
    )
    obs, _ = render_env.reset()
    done = False
    while not done:
        with torch.no_grad():
            action, *_ = agent.get_action_and_value(
                torch.tensor(obs).unsqueeze(0).to(device)
            )
        obs, _, terminated, truncated, _ = render_env.step(
            action.cpu().numpy().squeeze()
        )
        done = terminated or truncated
    render_env.close()
```

### Optional W&B Integration

```python
if args.track_wandb:
    import wandb
    wandb.init(project=args.wandb_project, name=run_name,
               config=dataclasses.asdict(args), sync_tensorboard=True)
```

---

## 8. Phase 4: Domain Randomization

Domain randomization is the primary tool for sim-to-real transfer. Apply it at every `reset()`.

### Parameters and Ranges

| Parameter | MuJoCo Field | Range | Rationale |
|---|---|---|---|
| Wheel/floor friction | `model.geom_friction` | ±40% of nominal | Wheel slip on different surfaces |
| Body masses | `model.body_mass` | ±20% per body | Battery level changes CoM |
| Joint damping | `model.dof_damping` | ±30% | BLDC cogging, back-EMF variation |
| Motor torque gain | `model.actuator_gear` | ±20% | SimpleFOC current loop calibration error |
| IMU angle noise | Added in `_get_obs()` | σ = 0.01 rad | nRF52840 IMU noise floor |
| IMU rate noise | Added in `_get_obs()` | σ = 0.05 rad/s | Gyro noise |
| **Action latency** | Ring buffer in `step()` | 1–3 steps (10–30ms) | BLE + compute delay — often the most critical |
| CoM offset | `model.body_ipos` | ±5mm | Uneven weight distribution |

**Critical:** After changing mass/inertia fields, always call:
```python
mujoco.mj_setConst(self.model, self.data)
```

### Action Latency Buffer

This is often the most impactful randomization for real hardware:

```python
def __init__(self, ...):
    self._action_delay_steps = 2        # nominal 2 steps * 10ms = 20ms
    self._action_buffer = np.zeros((5, 2))  # ring buffer

def step(self, action):
    delay = self.np_random.integers(1, self._action_delay_steps + 1)
    self._action_buffer = np.roll(self._action_buffer, 1, axis=0)
    self._action_buffer[0] = action
    delayed_action = self._action_buffer[delay]
    self.data.ctrl[:] = np.clip(delayed_action, -1.0, 1.0)
    # ... rest of step
```

### Curriculum Schedule

Start training **without randomization** until the policy reliably balances, then linearly increase randomization ranges:

```python
def get_randomization_scale(update, total_updates):
    """Ramp from 10% to 100% of nominal randomization ranges over first half of training."""
    return min(1.0, 0.1 + 0.9 * (2.0 * update / total_updates))
```

---

## 9. Phase 5: Deploying to Arduino Nano 33 BLE

### Hardware Budget (nRF52840: 1MB Flash, 256KB RAM, Cortex-M4 @ 64MHz)

For a 3-layer MLP with architecture 9 → 64 → 64 → 2:

| Representation | Parameters | Flash | Inference RAM |
|---|---|---|---|
| float32 | 4,930 | ~20 KB | 512 bytes |
| int8 quantized | 4,930 | ~5 KB | 128 bytes |

Both are trivially small. No network compression needed.

### Option A: Manual C++ Inference (Recommended First)

No library dependencies. Zero overhead. Validate numerical match with Python before moving to TFLite.

**Export script (`export_weights.py`):**
```python
import torch
import numpy as np

def tensor_to_c_array(t, name):
    arr = t.detach().cpu().numpy()
    shape_str = "][".join(str(d) for d in arr.shape)
    vals = ", ".join(f"{v:.8f}f" for v in arr.flatten())
    return f"static const float {name}[{shape_str}] = {{{vals}}};\n"

# Load trained agent
agent = torch.load("models/final.pt", map_location="cpu")
# actor_mean is Sequential([Linear, Tanh, Linear, Tanh, Linear])
layers = [l for l in agent.actor_mean if isinstance(l, torch.nn.Linear)]

with open("firmware/policy_weights.h", "w") as f:
    f.write("#pragma once\n\n")
    for i, layer in enumerate(layers, start=1):
        f.write(tensor_to_c_array(layer.weight, f"W{i}"))
        f.write(tensor_to_c_array(layer.bias,   f"b{i}"))

print("Exported firmware/policy_weights.h")
```

**C++ inference (`firmware/policy.h`):**
```cpp
#pragma once
#include <math.h>
#include "policy_weights.h"  // generated by export_weights.py

// Network: 9 -> 64 -> 64 -> 2, activations: Tanh, Tanh, Tanh
void policy_forward(const float obs[9], float action[2]) {
    float h1[64], h2[64];

    // Layer 1: h1 = tanh(W1 @ obs + b1)
    for (int i = 0; i < 64; i++) {
        float sum = b1[i];
        for (int j = 0; j < 9; j++) sum += W1[i][j] * obs[j];
        h1[i] = tanhf(sum);
    }

    // Layer 2: h2 = tanh(W2 @ h1 + b2)
    for (int i = 0; i < 64; i++) {
        float sum = b2[i];
        for (int j = 0; j < 64; j++) sum += W2[i][j] * h1[j];
        h2[i] = tanhf(sum);
    }

    // Output: action = tanh(W3 @ h2 + b3), squashed to [-1, 1]
    for (int i = 0; i < 2; i++) {
        float sum = b3[i];
        for (int j = 0; j < 64; j++) sum += W3[i][j] * h2[j];
        action[i] = tanhf(sum);
    }
}
```

**Arduino sketch (`firmware/policy_inference.ino`):**
```cpp
#include "policy.h"
// ... IMU and SimpleFOC setup ...

void loop() {
    // Populate observation from IMU and wheel encoders
    float obs[9];
    obs[0] = imu.getPitch();        // rad
    obs[1] = imu.getPitchRate();    // rad/s
    obs[2] = imu.getRoll();         // rad
    obs[3] = imu.getRollRate();     // rad/s
    obs[4] = motor_left.shaft_velocity;   // rad/s
    obs[5] = motor_right.shaft_velocity;  // rad/s
    obs[6] = 0.0f;  // x_vel: not directly measured; can estimate from wheel avg
    obs[7] = 0.0f;  // y_vel: not measured
    obs[8] = imu.getYawRate();      // rad/s

    // Run policy
    float action[2];
    policy_forward(obs, action);

    // action is in [-1, 1]; scale to motor torque command
    motor_left.move(action[0]  * MAX_TORQUE_NM);
    motor_right.move(action[1] * MAX_TORQUE_NM);

    delayMicroseconds(10000);  // 100 Hz control loop
}
```

### Option B: TFLite Micro (Better for Long-Term Maintainability)

Use if you want int8 quantization benefits or the CMSIS-NN backend (accelerated int8 matmuls on Cortex-M4 DSP).

**Export pipeline (PyTorch → ONNX → TFLite int8):**
```python
# Step 1: Export to ONNX
import torch
dummy = torch.zeros(1, 9)
torch.onnx.export(agent.actor_mean, dummy, "policy.onnx",
                  input_names=["obs"], output_names=["action"],
                  opset_version=13)

# Step 2: ONNX -> TFLite with int8 quantization
# pip install onnx2tf
import onnx2tf
onnx2tf.convert(
    input_onnx_file_path="policy.onnx",
    output_folder_path="tflite_model",
    quant_type="full_int8",
)

# Step 3: Convert to C array
# xxd -i tflite_model/policy_full_int8.tflite > firmware/policy_model_data.h
```

Arduino sketch uses `Arduino_TensorFlowLite` library (or the maintained `Chirale_TensorFlowLite` fork):
```cpp
#include <TensorFlowLite.h>
#include "policy_model_data.h"

constexpr int kTensorArenaSize = 20 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
// ... standard TFLite Micro setup + invoke pattern ...
```

### Observation Normalization (Critical)

Observations in simulation may not be on the same scale as real sensor outputs. Add normalization in the training script and match it exactly on the Arduino:

```python
# Training: normalize observations before feeding to network
obs_mean = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0.])  # update from data
obs_std  = np.array([0.3, 2.0, 0.3, 2.0, 10.0, 10.0, 1.0, 1.0, 2.0])
normalized_obs = (obs - obs_mean) / obs_std
```

```cpp
// Arduino: same normalization
static const float OBS_MEAN[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
static const float OBS_STD[9]  = {0.3, 2.0, 0.3, 2.0, 10.0, 10.0, 1.0, 1.0, 2.0};
for (int i = 0; i < 9; i++) obs[i] = (obs[i] - OBS_MEAN[i]) / OBS_STD[i];
```

---

## 10. Implementation Sequence

### Step-by-Step Plan

**Step 1: Environment validation**
- Implement MJCF + Gymnasium env, disabled domain randomization
- Verify simulation is stable with zero actions (robot should fall naturally)
- Verify it falls within expected timeframe (~1s)
- Use `mujoco.viewer.launch_passive()` to visually inspect the model

**Step 2: PPO baseline (no domain randomization)**
- Copy CleanRL `ppo_continuous_action.py`, adapt for `TwoWheelBalancer-v0`
- Add checkpointing and periodic video
- Train to convergence (~1–5M steps, ~30min on a modern CPU with 8 envs)
- Inspect reward curves for sign of learning

**Step 3: Domain randomization**
- Enable randomization one parameter at a time
- Verify policy still converges with each addition
- Start with ±10% ranges, then expand to the full ranges above
- Action latency is usually the hardest — add last

**Step 4: Switch to SAC for efficiency**
- Copy CleanRL `sac_continuous_action.py`
- Expect ~5x faster convergence
- Compare final performance against PPO

**Step 5: Weight export and C++ validation**
- Run `export_weights.py` to generate `policy_weights.h`
- Write a Python script that runs both the PyTorch model and the manual C++ inference (compiled via `cffi` or just test in a separate C program)
- Assert numerical agreement to within 1e-5

**Step 6: Hardware deployment**
- Flash firmware to Nano 33 BLE
- Log real robot observations during PID operation; compare distributions to simulation
- Tune `MAX_TORQUE_NM` scaling
- Enable the RL policy; expect to iterate on observation normalization and action scaling

### Key Gotchas

| Issue | Solution |
|---|---|
| Policy learned in sim doesn't transfer | Check obs normalization, action scaling, and latency modeling |
| IMU pitch convention mismatch | Log raw IMU values during PID op; verify sign conventions match MJCF |
| Action latency causes instability | Model 1–3 step delay in sim; add `delayMicroseconds` between sense and act |
| Reward function too sparse | Use the shaped reward above; avoid sparse ±1 rewards |
| Policy exploits sim artifacts | Add more aggressive domain randomization; randomize gravity slightly |

---

## 11. Sources

- [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Modeling Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [Gymnasium MuJoCo Environments](https://gymnasium.farama.org/environments/mujoco/)
- [CleanRL GitHub](https://github.com/vwxyzjn/cleanrl)
- [CleanRL PPO Continuous Action](https://github.com/vwxyzjn/cleanrl/blob/master/cleanrl/ppo_continuous_action.py)
- [CleanRL SAC Continuous Action](https://github.com/vwxyzjn/cleanrl/blob/master/cleanrl/sac_continuous_action.py)
- [CleanRL Resume Training Docs](https://docs.cleanrl.dev/advanced/resume-training/)
- [PPO vs SAC vs TD3 for locomotion tasks](https://www.scirp.org/journal/paperinformation?paperid=131938)
- [LiteRT (TFLite) for Microcontrollers](https://ai.google.dev/edge/litert/microcontrollers/overview)
- [TFLite Micro on Arduino Nano 33 BLE](https://forum.arduino.cc/t/tensorflow-lite-where-is-the-model-during-runtime-in-arduino-nano-33-ble-ram-or-flash/850823)
- [CMSIS-NN accelerated inference on ARM Cortex-M](https://blog.tensorflow.org/2021/02/accelerated-inference-on-arm-microcontrollers-with-tensorflow-lite.html)
- [mjlab PyPI](https://pypi.org/project/mjlab/)
- [mjlab GitHub](https://github.com/mujocolab/mjlab)
- [mjlab_upkie: wheeled biped RL on mjlab (close reference)](https://github.com/MarcDcls/mjlab_upkie)
- [RSL-RL (on-policy PPO used by mjlab)](https://github.com/leggedrobotics/rsl_rl)
- [MuJoCo Playground (MJX + JAX RL framework)](https://github.com/google-deepmind/mujoco_playground)
- [MuJoCo XLA (MJX) Documentation](https://mujoco.readthedocs.io/en/stable/mjx.html)
- [MuJoCo Menagerie (reference robot models)](https://github.com/google-deepmind/mujoco_menagerie)
