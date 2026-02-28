"""Tests for utility scripts in scripts/.

Each script exposes a load_scene() (or equivalent) function that sets up the
simulation state without opening a viewer, so these tests run headlessly.
"""

import importlib.util
import sys
from pathlib import Path

import mujoco
import pytest

SCRIPTS_DIR = Path(__file__).parents[1] / "scripts"


def import_script(name: str):
    """Import a script from scripts/ by filename stem."""
    path = SCRIPTS_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ── view_model ────────────────────────────────────────────────────────────────

class TestViewModelScript:
    def test_importable(self):
        """Script loads without error and without opening a window."""
        mod = import_script("view_model")
        assert hasattr(mod, "load_scene")
        assert hasattr(mod, "main")

    def test_load_scene_returns_model_and_data(self):
        mod = import_script("view_model")
        model, data = mod.load_scene()
        assert isinstance(model, mujoco.MjModel)
        assert isinstance(data, mujoco.MjData)

    def test_load_scene_upright_pose(self):
        mod = import_script("view_model")
        model, data = mod.load_scene()
        # Chassis should be at rest height, upright quaternion
        assert abs(data.qpos[2] - 0.085) < 1e-6
        assert abs(data.qpos[3] - 1.0) < 1e-6   # quat w ≈ 1 (no rotation)

    def test_load_scene_custom_xml(self, tmp_path):
        """load_scene() accepts an explicit path argument."""
        mod = import_script("view_model")
        default_xml = mod.XML_PATH
        model, data = mod.load_scene(xml_path=default_xml)
        assert model.nbody == 4

    def test_xml_path_exists(self):
        mod = import_script("view_model")
        assert mod.XML_PATH.exists(), f"XML_PATH {mod.XML_PATH} not found"
