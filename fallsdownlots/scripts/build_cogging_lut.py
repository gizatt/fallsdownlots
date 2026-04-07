"""
Build cogging feedforward LUT from calibration data.

Usage:
    # From a dedicated cogging sweep:
    python scripts/build_cogging_lut.py <cogging.csv> <r|l>

    # From a regular sysid CSV (uses low-speed velocity_steps):
    python scripts/build_cogging_lut.py <sysid.csv> <r|l>

The script auto-detects the source by checking which test labels are present.
A sysid CSV must have been collected with firmware that streams voltage_q
(the 5-field format: t_us, angle_rad, vel_rad_s, cmd, voltage_q).

Algorithm:
  1. Separate forward (step_target > 0) and reverse (step_target < 0) samples.
  2. Compute electrical angle: (angle_rad * N_POLE_PAIRS) % (2π).
  3. Bin voltage_q by electrical angle into N_LUT equal bins.
  4. Average the per-bin means from both passes — friction and back-EMF
     (both direction-dependent) cancel; position-dependent cogging remains.
  5. Subtract the overall mean (DC handled by the PID integrator).
  6. Emit the C header.

The feedforward is applied as:
    motor.voltage.q += lut_interp(motor.electricalAngle(), COGGING_LUT, 128)
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


N_POLE_PAIRS  = 7
N_LUT         = 128
MAX_SPEED_SYSID = 8.0  # rad/s — only low-speed steps have strong cogging signal


def load(path: str) -> pd.DataFrame:
    """
    Load and return a DataFrame with columns: step_target, angle_rad, voltage_q.

    Accepts either a dedicated cogging sweep CSV (test == "cogging_sweep") or a
    regular sysid CSV (test == "velocity_steps").  For the sysid case, only
    low-speed steps are used and only the steady-state tail of each step.
    """
    df = pd.read_csv(path)
    tests = set(df["test"].unique())

    if "cogging_sweep" in tests:
        cal = df[df["test"] == "cogging_sweep"].copy()
        n_fwd = (cal["step_target"] > 0).sum()
        n_rev = (cal["step_target"] < 0).sum()
        print(f"[info] source: cogging sweep  {len(cal)} rows  "
              f"({n_fwd} fwd, {n_rev} rev)", file=sys.stderr)

    elif "velocity_steps" in tests:
        sub = df[(df["test"] == "velocity_steps") &
                 (df["step_target"].abs() <= MAX_SPEED_SYSID)].copy()
        if sub.empty:
            sys.exit(f"ERROR: no velocity_steps rows with |speed| ≤ "
                     f"{MAX_SPEED_SYSID} rad/s found in {path}")
        if "voltage_q" not in sub.columns:
            sys.exit("ERROR: voltage_q column missing — re-run sysid with "
                     "updated firmware that streams voltage.q")
        # Keep only the steady-state tail of each step to avoid transients.
        chunks = [grp.iloc[len(grp) // 2:]
                  for _, grp in sub.groupby("step_target")]
        cal = pd.concat(chunks)
        n_fwd = (cal["step_target"] > 0).sum()
        n_rev = (cal["step_target"] < 0).sum()
        speeds = sorted(cal["step_target"].abs().unique())
        print(f"[info] source: sysid velocity_steps  {len(cal)} rows  "
              f"({n_fwd} fwd, {n_rev} rev)  speeds={speeds}", file=sys.stderr)

    else:
        sys.exit(f"ERROR: no 'cogging_sweep' or 'velocity_steps' rows in {path}")

    n_fwd = (cal["step_target"] > 0).sum()
    n_rev = (cal["step_target"] < 0).sum()
    if n_fwd == 0 or n_rev == 0:
        sys.exit("ERROR: need both forward (step_target > 0) and reverse rows.")

    return cal


def build_lut(cal: pd.DataFrame) -> np.ndarray:
    fwd = cal[cal["step_target"] > 0]
    rev = cal[cal["step_target"] < 0]

    # Electrical angle in [0, 2π).  angle_rad is SimpleFOC's accumulated angle
    # (no wrapping needed), so just scale and mod.
    elec_fwd = (fwd["angle_rad"].values * N_POLE_PAIRS) % (2 * np.pi)
    elec_rev = (rev["angle_rad"].values * N_POLE_PAIRS) % (2 * np.pi)
    vq_fwd   = fwd["voltage_q"].values
    vq_rev   = rev["voltage_q"].values

    bin_edges = np.linspace(0, 2 * np.pi, N_LUT + 1)

    def bin_mean(elec: np.ndarray, vq: np.ndarray) -> np.ndarray:
        means = np.full(N_LUT, np.nan)
        for i in range(N_LUT):
            mask = (elec >= bin_edges[i]) & (elec < bin_edges[i + 1])
            if mask.any():
                means[i] = vq[mask].mean()
        n_empty = np.isnan(means).sum()
        if n_empty > 0:
            print(f"[warn] {n_empty}/{N_LUT} bins empty — interpolating",
                  file=sys.stderr)
            idx   = np.arange(N_LUT)
            valid = ~np.isnan(means)
            means = np.interp(idx, idx[valid], means[valid], period=N_LUT)
        return means

    mean_fwd = bin_mean(elec_fwd, vq_fwd)
    mean_rev = bin_mean(elec_rev, vq_rev)

    # Average passes: friction and K_e·v (both direction-dependent) cancel.
    lut = 0.5 * (mean_fwd + mean_rev)
    lut -= lut.mean()   # DC handled by PID integrator

    rms_mv = np.sqrt(np.mean(lut ** 2)) * 1000
    pk_mv  = (lut.max() - lut.min()) * 1000
    print(f"[info] cogging LUT  RMS={rms_mv:.1f} mV  peak-to-peak={pk_mv:.1f} mV",
          file=sys.stderr)

    return lut


def emit_header(lut: np.ndarray, motor: str, out_path: Path) -> None:
    tag = motor.upper()
    lines = [
        f"// {motor.capitalize()} motor cogging feedforward LUT.",
        f"// Generated by: python scripts/build_cogging_lut.py <cal.csv> {motor}",
        f"// Indexed by electrical angle in [0, 2pi), 128 points, values in volts.",
        f"// Do not edit by hand — regenerate from calibration data.",
        f"#pragma once",
        f"",
        f"static constexpr float COGGING_LUT_{tag}[{N_LUT}] = {{",
    ]
    for i, v in enumerate(lut):
        comma = "," if i < N_LUT - 1 else ""
        lines.append(f"  {v:+.6f}f{comma}")
    lines.append("};")
    lines.append("")

    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[info] wrote {out_path}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Build cogging feedforward LUT from calibration CSV")
    parser.add_argument("csv",   help="Cogging sweep or sysid CSV")
    parser.add_argument("motor", choices=["r", "l"], help="Motor side (r=right, l=left)")
    args = parser.parse_args()

    src_dir  = Path(__file__).parent.parent / "src"
    out_path = src_dir / f"cogging_lut_{args.motor}.h"

    cal = load(args.csv)
    lut = build_lut(cal)
    emit_header(lut, args.motor, out_path)


if __name__ == "__main__":
    main()
