"""
Diagnostic plots for encoder LUT calibration data.

Usage:
    uv run scripts/plot_encoder_cal.py <cal.csv>

Expects a CSV produced by:
    python fallsdownlots/scripts/sysid.py --calibrate --out cal.csv

Produces three figures:

  1. Raw error traces
     Forward and backward pass encoder errors (raw_mech - ref_mech) vs
     mechanical reference angle.  Good data: the two traces are parallel
     (symmetric offset = friction); the slow wave is eccentricity.
     Bad data: traces diverge in shape (motor skipped steps), or raw_mech
     decreases while ref_mech increases (phase ordering wrong).

  2. Signal decomposition
     Averaged error (friction cancelled), the lowpass-filtered version
     (= what the LUT corrects), and the high-frequency remainder
     (= cogging + noise, intentionally not corrected).

  3. Correction quality
     Before and after: residual error vs reference angle using the
     uncorrected and LUT-corrected sensor readings.  The corrected
     residual should be much smaller in amplitude and contain only
     high-frequency cogging oscillations.
"""

import argparse
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal


N_POLE_PAIRS = 7
N_LUT        = 128
N_GRID       = 4096
F_CUT_CYCS   = 3.0   # lowpass cutoff: cycles per mechanical rotation


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    cal = df[df["test"] == "encoder_cal"].copy()
    if cal.empty:
        sys.exit(f"ERROR: no 'encoder_cal' rows found in {path}")
    print(f"[info] {len(cal)} calibration rows  "
          f"({(cal['direction']=='F').sum()} fwd, "
          f"{(cal['direction']=='R').sum()} rev)")
    return cal


def process(cal: pd.DataFrame):
    """Return all intermediate arrays used for plotting and LUT building."""
    fwd = cal[cal["direction"] == "F"]
    bwd = cal[cal["direction"] == "R"]

    ref_fwd = np.unwrap(fwd["ref_elec"].values / N_POLE_PAIRS)
    raw_fwd = np.unwrap(fwd["raw_mech"].values)
    ref_bwd = np.unwrap(bwd["ref_elec"].values / N_POLE_PAIRS)
    raw_bwd = np.unwrap(bwd["raw_mech"].values)

    err_fwd = raw_fwd - ref_fwd
    err_bwd = raw_bwd - ref_bwd

    # Reverse bwd so both run low→high ref_mech.
    ref_bwd_r = ref_bwd[::-1]
    err_bwd_r = err_bwd[::-1]

    rot_start = max(ref_fwd[0],  ref_bwd_r[0])
    rot_end   = min(ref_fwd[-1], ref_bwd_r[-1])
    grid = np.linspace(rot_start, rot_end, N_GRID)

    err_f_interp = np.interp(grid, ref_fwd,   err_fwd)
    err_r_interp = np.interp(grid, ref_bwd_r, err_bwd_r)
    avg = 0.5 * (err_f_interp + err_r_interp)

    Wn = F_CUT_CYCS / (N_GRID / 2)
    b, a = signal.butter(4, Wn)
    smooth = signal.filtfilt(b, a, avg)
    smooth -= smooth.mean()

    lut_ref    = np.linspace(0, 2 * np.pi, N_LUT, endpoint=False)
    lut_values = np.interp(lut_ref, grid % (2 * np.pi), smooth, period=2 * np.pi)

    return dict(
        ref_fwd=ref_fwd, err_fwd=err_fwd,
        ref_bwd=ref_bwd_r, err_bwd=err_bwd_r,
        grid=grid,
        err_f_interp=err_f_interp,
        err_r_interp=err_r_interp,
        avg=avg,
        smooth=smooth,
        lut_ref=lut_ref,
        lut_values=lut_values,
    )


def apply_lut(raw: np.ndarray, lut_ref: np.ndarray, lut_values: np.ndarray) -> np.ndarray:
    """Apply the LUT correction to an array of raw angles."""
    pos = raw * N_LUT / (2 * np.pi)
    i0  = pos.astype(int) % N_LUT
    i1  = (i0 + 1) % N_LUT
    f   = pos - pos.astype(int)
    corr = lut_values[i0] + f * (lut_values[i1] - lut_values[i0])
    return raw - corr


def figure_raw_traces(d: dict):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Raw calibration error traces", fontsize=13)

    # Left: error vs reference angle for both passes.
    ax = axes[0]
    ax.plot(d["ref_fwd"], d["err_fwd"] * 1e3, lw=0.8, alpha=0.8, label="forward")
    ax.plot(d["ref_bwd"], d["err_bwd"] * 1e3, lw=0.8, alpha=0.8, label="reverse")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("reference mechanical angle (rad)")
    ax.set_ylabel("error  raw − ref  (mrad)")
    ax.set_title("Error per pass\n"
                 "parallel traces → friction cancels cleanly\n"
                 "decreasing raw → swap two motor wires")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Right: raw_mech vs ref_mech — should be a straight line of slope 1.
    ax = axes[1]
    ax.plot(d["ref_fwd"] % (2 * np.pi), (d["ref_fwd"] + d["err_fwd"]) % (2 * np.pi),
            ",", alpha=0.3, label="forward")
    ax.plot(d["ref_bwd"] % (2 * np.pi), (d["ref_bwd"] + d["err_bwd"]) % (2 * np.pi),
            ",", alpha=0.3, label="reverse")
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(th, th, "k--", lw=1, alpha=0.5, label="ideal (slope 1)")
    ax.set_xlabel("reference mechanical angle (rad, mod 2π)")
    ax.set_ylabel("raw sensor angle (rad, mod 2π)")
    ax.set_title("Sensor vs reference\nshould lie near the diagonal")
    ax.legend(fontsize=9, markerscale=10)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()


def figure_decomposition(d: dict):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle("Signal decomposition", fontsize=13)

    grid_deg = np.degrees(d["grid"])

    # Top: raw fwd/bwd and their average.
    ax = axes[0]
    ax.plot(grid_deg, d["err_f_interp"] * 1e3, lw=0.6, alpha=0.5, label="forward (interp)")
    ax.plot(grid_deg, d["err_r_interp"] * 1e3, lw=0.6, alpha=0.5, label="reverse (interp)")
    ax.plot(grid_deg, d["avg"] * 1e3, lw=1.5, color="k", label="average (friction cancelled)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("error (mrad)")
    ax.set_title("Forward, reverse, and average")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Middle: averaged vs LUT (lowpassed).
    ax = axes[1]
    ax.plot(grid_deg, d["avg"] * 1e3, lw=0.8, alpha=0.6,
            label=f"average (all content)")
    ax.plot(grid_deg, d["smooth"] * 1e3, lw=2.0, color="C2",
            label=f"LUT — lowpass < {F_CUT_CYCS:.0f} cyc/rot (eccentricity only)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("error (mrad)")
    ax.set_title("Average vs LUT correction")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Bottom: high-frequency remainder = cogging (not corrected by LUT).
    ax = axes[2]
    remainder = d["avg"] - d["smooth"]
    ax.plot(grid_deg, remainder * 1e3, lw=0.8, color="C3",
            label=f"remainder (cogging, > {F_CUT_CYCS:.0f} cyc/rot)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("reference mechanical angle (deg)")
    ax.set_ylabel("error (mrad)")
    ax.set_title("Cogging residual (not corrected)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    rms_ecc  = np.sqrt(np.mean(d["smooth"] ** 2)) * 1e3
    rms_cog  = np.sqrt(np.mean(remainder ** 2)) * 1e3
    print(f"[info] eccentricity RMS: {rms_ecc:.2f} mrad")
    print(f"[info] cogging RMS:      {rms_cog:.2f} mrad")

    fig.tight_layout()


def figure_correction_quality(d: dict):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Correction quality: error before and after LUT", fontsize=13)

    ref_fwd = d["ref_fwd"]
    err_fwd = d["err_fwd"]
    raw_fwd = ref_fwd + err_fwd  # unwrapped raw angle

    # Apply LUT to the raw angle.
    corrected_fwd = apply_lut(raw_fwd % (2 * np.pi), d["lut_ref"], d["lut_values"])
    # Unwrap corrected back to compare with ref.
    corrected_fwd_u = np.unwrap(corrected_fwd) + (raw_fwd - np.unwrap(raw_fwd % (2 * np.pi)))
    err_corrected = corrected_fwd_u - ref_fwd

    ax = axes[0]
    ax.plot(ref_fwd % (2 * np.pi), err_fwd * 1e3, ",", alpha=0.3, color="C0")
    ax.plot(ref_fwd % (2 * np.pi), err_corrected * 1e3, ",", alpha=0.3, color="C2")
    # Dummy lines for legend.
    ax.plot([], [], color="C0", label=f"before  RMS={np.sqrt(np.mean(err_fwd**2))*1e3:.2f} mrad")
    ax.plot([], [], color="C2", label=f"after   RMS={np.sqrt(np.mean(err_corrected**2))*1e3:.2f} mrad")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("reference mechanical angle (rad, mod 2π)")
    ax.set_ylabel("error (mrad)")
    ax.set_title("Scatter: error before (blue) and after (green) LUT")
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    # Right: LUT shape.
    ax = axes[1]
    lut_deg = np.degrees(d["lut_ref"])
    ax.plot(lut_deg, d["lut_values"] * 1e3, lw=2, color="C2")
    ax.axhline(0, color="gray", lw=0.5)
    pk = (d["lut_values"].max() - d["lut_values"].min()) * 1e3
    ax.set_xlabel("mechanical angle (deg)")
    ax.set_ylabel("correction (mrad)")
    ax.set_title(f"LUT correction curve  (peak-to-peak = {pk:.2f} mrad)\n"
                 "smooth sinusoid → eccentricity; jagged → data problem")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(description="Plot encoder calibration diagnostics")
    parser.add_argument("csv", help="Calibration CSV from sysid.py --calibrate")
    args = parser.parse_args()

    cal = load(args.csv)
    d   = process(cal)

    figure_raw_traces(d)
    figure_decomposition(d)
    figure_correction_quality(d)

    plt.show()


if __name__ == "__main__":
    main()
