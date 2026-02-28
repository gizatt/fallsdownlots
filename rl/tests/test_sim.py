"""Basic simulation sanity tests for the two-wheel balancer model."""

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

XML_PATH = Path(__file__).parents[1] / "src/two_wheel_mjlab/robot/two_wheel/two_wheel.xml"


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(XML_PATH))


@pytest.fixture
def data(model):
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    return d


# ── Structure ─────────────────────────────────────────────────────────────────

def test_model_loads():
    mujoco.MjModel.from_xml_path(str(XML_PATH))


def test_structure(model):
    assert model.nbody == 4        # world + chassis + 2 wheels
    assert model.njnt  == 3        # 1 freejoint + 2 hinges
    assert model.nu    == 2        # left + right motor
    assert model.nsensor == 4      # gyro, quat, left_vel, right_vel


def test_body_names(model):
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]
    assert "chassis"     in names
    assert "left_wheel"  in names
    assert "right_wheel" in names


def test_total_mass(model):
    total_kg = model.body_mass.sum()
    # chassis 80g + 2 × 20g wheels = 120g
    assert math.isclose(total_kg, 0.120, abs_tol=1e-6)


# ── Geometry / resting pose ───────────────────────────────────────────────────

def test_resting_height(data):
    # Chassis centre should be at wheel_radius + chassis_half_z = 0.035 + 0.050 = 0.085 m
    assert math.isclose(data.qpos[2], 0.085, abs_tol=1e-6)


def test_wheel_floor_contact(model, data):
    # Both wheels should touch the floor at rest
    assert data.ncon >= 2
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    wheel_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_wheel_geom"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_wheel_geom"),
    }
    contacting_wheels = set()
    for i in range(data.ncon):
        c = data.contact[i]
        if c.geom1 == floor_id and c.geom2 in wheel_ids:
            contacting_wheels.add(c.geom2)
        if c.geom2 == floor_id and c.geom1 in wheel_ids:
            contacting_wheels.add(c.geom1)
    assert contacting_wheels == wheel_ids, "Both wheels must touch the floor at rest"


# ── Physics ───────────────────────────────────────────────────────────────────

def test_no_nan_in_simulation(model, data):
    """500 steps with zero torque must not produce NaN/inf anywhere."""
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos)), "qpos contains NaN/inf"
    assert np.all(np.isfinite(data.qvel)), "qvel contains NaN/inf"


def test_robot_falls_without_torque(model, data):
    """A 5° forward pitch should cause a clear fall within 0.6 s.

    Pitch is rotation around the Y axis: qpos[5] = sin(θ/2), qpos[3] = cos(θ/2).
    (qpos[4] is roll/X-axis, qpos[6] is yaw/Z-axis — those don't cause a forward fall.)
    """
    theta = math.radians(5)
    data.qpos[3] = math.cos(theta / 2)   # quat w
    data.qpos[5] = math.sin(theta / 2)   # quat y  (forward pitch)
    mujoco.mj_forward(model, data)

    for _ in range(300):   # 0.6 s
        mujoco.mj_step(model, data)

    # Chassis should have dropped well below the upright rest height of 0.085 m
    assert data.qpos[2] < 0.060, "Robot should have fallen without control torque"


def test_torque_opposes_fall(model, data):
    """Forward wheel torque should slow a forward fall compared to zero torque.

    Positive torque drives the robot in +X (verified empirically); for a forward
    lean (positive pitch around Y) that's the corrective direction.
    """
    theta = math.radians(5)

    def run(torque, steps=200):
        d = mujoco.MjData(model)
        d.qpos[3] = math.cos(theta / 2)
        d.qpos[5] = math.sin(theta / 2)
        mujoco.mj_forward(model, d)
        for _ in range(steps):
            d.ctrl[0] = torque
            d.ctrl[1] = torque
            mujoco.mj_step(model, d)
        return d.qpos[2]

    z_free = run(torque=0.0)
    z_ctrl = run(torque=1.0)

    assert z_ctrl > z_free, (
        f"Forward torque (z={z_ctrl:.4f}) should keep chassis higher "
        f"than free-fall (z={z_free:.4f})"
    )
