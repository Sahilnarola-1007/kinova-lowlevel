#!/usr/bin/env python3
"""Estimate the joint servo's effective proportional gain K from a tracking log.

Model (INFERRED, not a documented Kortex control law):
    joint velocity  v = K * (q_send - q_meas) = K * gap

At steady state the joint moves at the demanded velocity, so
    q_dot_des = K * gap
and K is the through-origin least-squares slope of q_dot_des against gap.

Least squares is used rather than a peak ratio because a single scheduler stall
produces one outsized gap sample that biases a max-based estimate badly.

Usage:  python3 analysis/fit_k.py sinusoid_tracking.csv
"""
import sys
import numpy as np
import pandas as pd

AMPLITUDE_DEG = 5.0
FREQ_HZ = 0.2
DT_S = 1e-3


def main(path: str) -> None:
    d = pd.read_csv(path)
    w = 2.0 * np.pi * FREQ_HZ
    t = d["cycle"].to_numpy() * DT_S
    qd = AMPLITUDE_DEG * w * np.cos(w * t)          # demanded velocity [deg/s]
    gap = d["gap_deg"].to_numpy()                   # [deg]

    k = float((qd * gap).sum() / (gap ** 2).sum())

    # Coefficient of determination for the through-origin fit -- if this is low,
    # the proportional model does not describe the servo and K is meaningless.
    resid = qd - k * gap
    r2 = 1.0 - float((resid ** 2).sum() / (qd ** 2).sum())

    meas = d["q_meas_deg"].to_numpy()
    amp = 0.5 * (meas.max() - meas.min())

    print(f"rows                 : {len(d)}")
    print(f"K (least squares)    : {k:.2f} /s   (tau ~ {1000.0 / k:.1f} ms)")
    print(f"R^2 (through origin) : {r2:.4f}")
    print(f"RMS gap              : {np.sqrt((gap ** 2).mean()):.4f} deg")
    print(f"measured amplitude   : {amp:.4f} deg  "
          f"(leak {100.0 * (AMPLITUDE_DEG - amp) / AMPLITUDE_DEG:+.2f} %)")
    print(f"max |Euler drift|    : {np.abs(d['drift_deg']).max():.4f} deg")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "sinusoid_tracking.csv")
