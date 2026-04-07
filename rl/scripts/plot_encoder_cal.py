"""
Diagnostic plots for encoder LUT calibration data.

Usage:
    uv run scripts/plot_encoder_cal.py <cal.csv>

Expects a CSV produced by:
    python fallsdownlots/scripts/sysid.py --calibrate --out cal.csv

All angular error signals are DC-removed (the constant encoder-position
offset, which initFOC absorbs, is subtracted before any plot or RMS
calculation).  Every error axis is in mrad; every angle axis is in degrees.

Produces three figures:

  1. Raw error traces
     Forward and backward pass errors (raw − ref) vs reference angle,
     DC-removed.  Good data: parallel traces with a small friction gap;
     the slow wave is eccentricity.  Phase-ordering wrong if raw decreases
     while ref increases.

  2. Signal decomposition
     DC-removed average (friction cancelled), the lowpass-filtered LUT
     correction (eccentricity only), and the high-frequency cogging remainder
     that the LUT intentionally does not correct.

  3. Correction quality
     Scatter of error before and after applying the LUT vs reference angle,
     both DC-removed.  Corrected residual should be smaller and show only
     high-frequency cogging.  Right panel shows the LUT curve itself.
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
F_CUT_CYCS   = 3.0   # lowpass cutoff in cycles per mechanical rotation


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    cal = df[df["test"] == "encoder_cal"].copy()
    if cal.empty:
        sys.exit(f"ERROR: no 'encoder_cal' rows found in {path}")
    print(f"[info] {len(cal)} calibration rows  "
          f"({(cal['direction']=='F').sum()} fwd, "
          f"{(cal['direction']=='R').sum()} rev)")
    return cal


def process(cal: pd.DataFrame) -> dict:
    """
    Compute all intermediate signals used for plotting and LUT building.

    Angle conventions
    -----------------
    ref_fwd / ref_bwd : unwrapped mechanical reference angle, monotonically
        increasing from ~0 to ~2π (one full rotation).
    err_fwd / err_bwd : raw_mech − ref_mech, unwrapped, on the same 2π
        branch (see branch-alignment note below).
    dc : mean of the averaged error — the constant initial-position offset
        that initFOC absorbs.  Subtracted from all displayed error signals.
    avg_c : dc-removed average error (eccentricity + cogging, zero-mean).
    smooth : lowpassed avg_c — the LUT correction (eccentricity only, zero-mean).
    cogging : avg_c − smooth — what the LUT leaves uncorrected.
    """
    fwd = cal[cal["direction"] == "F"]
    bwd = cal[cal["direction"] == "R"]

    # Unwrap reference and raw angles for both passes.
    ref_fwd = np.unwrap(fwd["ref_elec"].values / N_POLE_PAIRS)
    raw_fwd = np.unwrap(fwd["raw_mech"].values)
    ref_bwd = np.unwrap(bwd["ref_elec"].values / N_POLE_PAIRS)
    raw_bwd = np.unwrap(bwd["raw_mech"].values)

    err_fwd = raw_fwd - ref_fwd
    err_bwd = raw_bwd - ref_bwd

    # Reverse the backward pass so both run low → high in ref angle.
    ref_bwd = ref_bwd[::-1]
    err_bwd = err_bwd[::-1]

    # The two passes unwrap in opposite directions and land on different 2π
    # branches (err_fwd ≈ +θ₀, err_bwd ≈ θ₀−2π when θ₀≈π → ±3000 mrad).
    # Snap bwd onto fwd's branch so the DC is shared and cancels cleanly.
    branch_offset = round((err_fwd.mean() - err_bwd.mean()) / (2 * np.pi)) * 2 * np.pi
    err_bwd = err_bwd + branch_offset

    # Interpolate both onto a uniform grid spanning one mechanical rotation.
    rot_start = max(ref_fwd[0], ref_bwd[0])
    rot_end   = min(ref_fwd[-1], ref_bwd[-1])
    grid = np.linspace(rot_start, rot_end, N_GRID)

    err_f_interp = np.interp(grid, ref_fwd, err_fwd)
    err_r_interp = np.interp(grid, ref_bwd, err_bwd)
    avg = 0.5 * (err_f_interp + err_r_interp)

    # DC = mean of the averaged error (initial encoder position, absorbed by
    # initFOC).  Remove it so all subsequent signals are zero-mean.
    dc = avg.mean()
    avg_c = avg - dc

    # Lowpass to isolate eccentricity (< F_CUT_CYCS cycles/rotation).
    # filtfilt on avg_c which is already zero-mean → smooth is also zero-mean.
    Wn = F_CUT_CYCS / (N_GRID / 2)
    b, a = signal.butter(4, Wn)
    smooth = signal.filtfilt(b, a, avg_c)

    cogging = avg_c - smooth

    # Resample to the 128-point LUT over [0, 2π).
    lut_ref    = np.linspace(0, 2 * np.pi, N_LUT, endpoint=False)
    lut_values = np.interp(lut_ref, grid % (2 * np.pi), smooth, period=2 * np.pi)

    return dict(
        # Per-pass (unwrapped, same branch)
        ref_fwd=ref_fwd, err_fwd=err_fwd,
        ref_bwd=ref_bwd, err_bwd=err_bwd,
        # Interpolated grid
        grid=grid,
        err_f_interp=err_f_interp,
        err_r_interp=err_r_interp,
        # Derived signals (all zero-mean)
        dc=dc,
        avg_c=avg_c,
        smooth=smooth,
        cogging=cogging,
        # LUT
        lut_ref=lut_ref,
        lut_values=lut_values,
    )


def lut_correction(raw_wrapped: np.ndarray, lut_ref: np.ndarray,
                   lut_values: np.ndarray) -> np.ndarray:
    """Interpolate LUT correction for raw angles in [0, 2π)."""
    pos  = raw_wrapped * N_LUT / (2 * np.pi)
    i0   = pos.astype(int) % N_LUT
    i1   = (i0 + 1) % N_LUT
    f    = pos - pos.astype(int)
    return lut_values[i0] + f * (lut_values[i1] - lut_values[i0])


def figure_raw_traces(d: dict):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Raw calibration error traces  (DC-removed)", fontsize=13)

    ref_fwd_deg = np.degrees(d["ref_fwd"] % (2 * np.pi))
    ref_bwd_deg = np.degrees(d["ref_bwd"] % (2 * np.pi))
    err_fwd_c   = (d["err_fwd"] - d["dc"]) * 1e3   # mrad, zero-mean
    err_bwd_c   = (d["err_bwd"] - d["dc"]) * 1e3

    # Left: error vs reference angle, both passes DC-removed.
    ax = axes[0]
    ax.plot(ref_fwd_deg, err_fwd_c, lw=0.8, alpha=0.8, label="forward")
    ax.plot(ref_bwd_deg, err_bwd_c, lw=0.8, alpha=0.8, label="reverse")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("reference mechanical angle (deg)")
    ax.set_ylabel("error  raw − ref  (mrad)")
    ax.set_title("Error per pass  (DC = initial encoder offset, removed)\n"
                 "parallel → friction cancels cleanly; slow wave → eccentricity\n"
                 "raw decreasing as ref increases → swap two motor wires")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Right: raw_mech vs ref_mech in [0°, 360°) — should lie near the diagonal.
    ax = axes[1]
    # raw angle in degrees (in [0°, 360°)), reconstructed from ref + error.
    raw_fwd_deg = np.degrees((d["ref_fwd"] + d["err_fwd"]) % (2 * np.pi))
    raw_bwd_deg = np.degrees((d["ref_bwd"] + d["err_bwd"]) % (2 * np.pi))
    ax.plot(ref_fwd_deg, raw_fwd_deg, ",", alpha=0.3, label="forward")
    ax.plot(ref_bwd_deg, raw_bwd_deg, ",", alpha=0.3, label="reverse")
    th = np.linspace(0, 360, 200)
    ax.plot(th, th, "k--", lw=1, alpha=0.5, label="ideal (slope 1)")
    ax.set_xlabel("reference mechanical angle (deg)")
    ax.set_ylabel("raw sensor angle (deg)")
    ax.set_title("Sensor vs reference\nshould lie near the diagonal")
    ax.legend(fontsize=9, markerscale=10)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()


def figure_decomposition(d: dict):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle("Signal decomposition  (all signals DC-removed, zero-mean)", fontsize=13)

    grid_deg = np.degrees(d["grid"])
    to_mrad  = 1e3

    # Top: per-pass errors and their friction-cancelled average (all DC-removed).
    ax = axes[0]
    ax.plot(grid_deg, (d["err_f_interp"] - d["dc"]) * to_mrad,
            lw=0.6, alpha=0.5, label="forward (interp)")
    ax.plot(grid_deg, (d["err_r_interp"] - d["dc"]) * to_mrad,
            lw=0.6, alpha=0.5, label="reverse (interp)")
    ax.plot(grid_deg, d["avg_c"] * to_mrad,
            lw=1.5, color="k", label="average (friction cancelled)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("error (mrad)")
    ax.set_title("Per-pass errors and friction-cancelled average")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Middle: averaged signal vs the LUT (both zero-mean).
    ax = axes[1]
    ax.plot(grid_deg, d["avg_c"] * to_mrad,
            lw=0.8, alpha=0.6, label="average (eccentricity + cogging)")
    ax.plot(grid_deg, d["smooth"] * to_mrad,
            lw=2.0, color="C2",
            label=f"LUT  (lowpass < {F_CUT_CYCS:.0f} cyc/rot — eccentricity only)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("error (mrad)")
    ax.set_title("Average vs LUT correction")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Bottom: cogging remainder (average minus LUT, zero-mean).
    ax = axes[2]
    ax.plot(grid_deg, d["cogging"] * to_mrad,
            lw=0.8, color="C3",
            label=f"cogging  (> {F_CUT_CYCS:.0f} cyc/rot — not corrected by LUT)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("reference mechanical angle (deg)")
    ax.set_ylabel("error (mrad)")
    ax.set_title("Cogging residual (intentionally uncorrected)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    rms_ecc = np.sqrt(np.mean(d["smooth"]  ** 2)) * to_mrad
    rms_cog = np.sqrt(np.mean(d["cogging"] ** 2)) * to_mrad
    print(f"[info] eccentricity RMS: {rms_ecc:.2f} mrad")
    print(f"[info] cogging RMS:      {rms_cog:.2f} mrad")

    fig.tight_layout()


def figure_correction_quality(d: dict):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Correction quality  (DC-removed)", fontsize=13)

    # err_fwd already on the same branch as avg; center it.
    ref_fwd = d["ref_fwd"]
    err_fwd_c = (d["err_fwd"] - d["dc"]) * 1e3  # mrad, zero-mean

    # LUT correction at each raw measurement point.
    raw_fwd_wrapped = (ref_fwd + d["err_fwd"]) % (2 * np.pi)
    corr = lut_correction(raw_fwd_wrapped, d["lut_ref"], d["lut_values"])
    # After correction the residual = err_fwd - lut_correction; DC-remove with same dc.
    err_after_c = (d["err_fwd"] - corr - d["dc"]) * 1e3  # mrad

    ref_fwd_deg = np.degrees(ref_fwd % (2 * np.pi))

    rms_before = np.sqrt(np.mean(err_fwd_c ** 2))
    rms_after  = np.sqrt(np.mean(err_after_c ** 2))

    ax = axes[0]
    ax.plot(ref_fwd_deg, err_fwd_c,  ",", alpha=0.3, color="C0")
    ax.plot(ref_fwd_deg, err_after_c, ",", alpha=0.3, color="C2")
    ax.plot([], [], color="C0", label=f"before  RMS = {rms_before:.2f} mrad")
    ax.plot([], [], color="C2", label=f"after   RMS = {rms_after:.2f} mrad")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("reference mechanical angle (deg)")
    ax.set_ylabel("error (mrad)")
    ax.set_title("Error before (blue) and after (green) LUT correction\n"
                 "after should be smaller amplitude, higher frequency only")
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
