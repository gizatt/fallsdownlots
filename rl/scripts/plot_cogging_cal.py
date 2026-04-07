"""
Diagnostic plots for cogging feedforward LUT calibration data.

Usage:
    uv run scripts/plot_cogging_cal.py <cogging.csv>

Expects a CSV produced by:
    python fallsdownlots/scripts/sysid.py --calibrate-cogging --out cogging.csv

Produces two figures:

  1. Raw voltage vs electrical angle
     Forward and reverse pass voltage.q plotted vs electrical angle (mod 2π).
     Good data: two offset curves (friction gap); similar shape → cogging
     dominates. If fwd and rev traces are mirrored (not offset), something else
     is going on (encoder error, current ripple, etc.).

  2. Cogging profile and LUT
     The friction-cancelled average (cogging) and the final LUT curve (which
     equals the average after mean subtraction). RMS and peak-to-peak printed.
"""

import argparse
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


N_POLE_PAIRS = 7
N_LUT        = 128


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    cal = df[df["test"] == "cogging_sweep"].copy()
    if cal.empty:
        sys.exit(f"ERROR: no 'cogging_sweep' rows found in {path}")
    n_fwd = (cal["step_target"] > 0).sum()
    n_rev = (cal["step_target"] < 0).sum()
    print(f"[info] {len(cal)} rows  ({n_fwd} fwd, {n_rev} rev)")
    return cal


def process(cal: pd.DataFrame) -> dict:
    fwd = cal[cal["step_target"] > 0].copy()
    rev = cal[cal["step_target"] < 0].copy()

    elec_fwd = (np.unwrap(fwd["angle_rad"].values) * N_POLE_PAIRS) % (2 * np.pi)
    elec_rev = (np.unwrap(rev["angle_rad"].values) * N_POLE_PAIRS) % (2 * np.pi)
    vq_fwd = fwd["voltage_q"].values
    vq_rev = rev["voltage_q"].values

    # Build per-bin means for both passes.
    bin_edges   = np.linspace(0, 2 * np.pi, N_LUT + 1)
    bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])

    def bin_mean(elec, vq):
        means = np.full(N_LUT, np.nan)
        for i in range(N_LUT):
            mask = (elec >= bin_edges[i]) & (elec < bin_edges[i + 1])
            if mask.any():
                means[i] = vq[mask].mean()
        valid = ~np.isnan(means)
        if not valid.all():
            idx = np.arange(N_LUT)
            means = np.interp(idx, idx[valid], means[valid], period=N_LUT)
        return means

    mean_fwd = bin_mean(elec_fwd, vq_fwd)
    mean_rev = bin_mean(elec_rev, vq_rev)

    avg = 0.5 * (mean_fwd + mean_rev)
    lut = avg - avg.mean()   # zero-mean feedforward

    return dict(
        elec_fwd=elec_fwd, vq_fwd=vq_fwd,
        elec_rev=elec_rev, vq_rev=vq_rev,
        bin_centers=bin_centers,
        mean_fwd=mean_fwd, mean_rev=mean_rev,
        avg=avg, lut=lut,
    )


def figure_raw(d: dict):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Raw voltage.q vs electrical angle", fontsize=13)

    ax = axes[0]
    ax.scatter(np.degrees(d["elec_fwd"]), d["vq_fwd"] * 1e3,
               s=1, alpha=0.15, color="C0", rasterized=True, label="forward")
    ax.scatter(np.degrees(d["elec_rev"]), d["vq_rev"] * 1e3,
               s=1, alpha=0.15, color="C1", rasterized=True, label="reverse")
    ax.plot(np.degrees(d["bin_centers"]), d["mean_fwd"] * 1e3,
            lw=2, color="C0", label="fwd bin mean")
    ax.plot(np.degrees(d["bin_centers"]), d["mean_rev"] * 1e3,
            lw=2, color="C1", label="rev bin mean")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("electrical angle (deg)")
    ax.set_ylabel("voltage.q  (mV)")
    ax.set_title("Per-pass voltage vs electrical angle\n"
                 "offset between fwd/rev = friction; shared shape = cogging")
    ax.legend(fontsize=9, markerscale=8)
    ax.grid(True, alpha=0.3)

    # Right: friction estimate = half the gap between passes.
    ax = axes[1]
    friction = 0.5 * (d["mean_fwd"] - d["mean_rev"])
    ax.plot(np.degrees(d["bin_centers"]), friction * 1e3, lw=1.5, color="C4")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("electrical angle (deg)")
    ax.set_ylabel("voltage  (mV)")
    ax.set_title("Friction estimate  = (fwd − rev) / 2\n"
                 "should be roughly constant (not angle-dependent)")
    ax.grid(True, alpha=0.3)

    friction_rms = np.sqrt(np.mean(friction ** 2)) * 1e3
    friction_pk  = (friction.max() - friction.min()) * 1e3
    print(f"[info] friction voltage  RMS={friction_rms:.1f} mV  "
          f"peak-to-peak={friction_pk:.1f} mV")

    fig.tight_layout()


def figure_lut(d: dict):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Cogging profile and feedforward LUT", fontsize=13)

    deg = np.degrees(d["bin_centers"])

    ax = axes[0]
    ax.plot(deg, d["avg"] * 1e3, lw=1.5, color="C3",
            label="avg (fwd+rev)/2 — cogging + DC")
    ax.plot(deg, d["lut"] * 1e3, lw=2, color="C2",
            label="LUT (zero-mean feedforward)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("electrical angle (deg)")
    ax.set_ylabel("voltage  (mV)")
    ax.set_title("Cogging profile\n"
                 "smooth sinusoid → clean; jagged → more revolutions needed")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    rms_mv = np.sqrt(np.mean(d["lut"] ** 2)) * 1e3
    pk_mv  = (d["lut"].max() - d["lut"].min()) * 1e3
    print(f"[info] cogging LUT  RMS={rms_mv:.1f} mV  peak-to-peak={pk_mv:.1f} mV")

    # Right: spectrum of the LUT (how many electrical cycles does it take?)
    ax = axes[1]
    spectrum = np.abs(np.fft.rfft(d["lut"]))
    freqs    = np.fft.rfftfreq(N_LUT, d=1.0 / N_LUT)  # cycles per electrical revolution
    ax.bar(freqs[1:], spectrum[1:], width=0.6, color="C2")
    ax.set_xlabel("harmonics (cycles per electrical revolution)")
    ax.set_ylabel("amplitude (linear)")
    ax.set_title("LUT spectrum\n"
                 "1× dominant → fundamental cogging; higher harmonics = slot structure")
    ax.set_xlim(0, 16)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="Plot cogging calibration diagnostics")
    parser.add_argument("csv", help="Cogging sweep CSV from sysid.py --calibrate-cogging")
    args = parser.parse_args()

    cal = load(args.csv)
    d   = process(cal)

    figure_raw(d)
    figure_lut(d)

    plt.show()


if __name__ == "__main__":
    main()
