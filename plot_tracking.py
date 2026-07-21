#!/usr/bin/env python3
"""Plot q_ideal / q_send / q_meas and the gap from a tracking log.

Usage:  python3 analysis/plot_tracking.py sinusoid_tracking.csv out.png
"""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

DT_S = 1e-3


def main(src: str, dst: str) -> None:
    d = pd.read_csv(src)
    t = d["cycle"] * DT_S

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(t, d["q_ideal_deg"], "--", lw=1.2, label="q_ideal")
    ax1.plot(t, d["q_send_deg"], lw=1.0, label="q_send")
    ax1.plot(t, d["q_meas_deg"], lw=1.0, label="q_meas")
    ax1.set_ylabel("joint 7 position [deg]")
    ax1.legend()
    ax1.grid(alpha=0.3)

    ax2.plot(t, d["gap_deg"], lw=0.8, label="gap = q_send - q_meas")
    ax2.plot(t, d["drift_deg"], lw=0.8, label="drift = q_send - q_ideal")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("[deg]")
    ax2.legend()
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(dst, dpi=130)
    print(f"wrote {dst}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "sinusoid_tracking.csv",
         sys.argv[2] if len(sys.argv) > 2 else "tracking.png")
