#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


R_EARTH = 6378137.0


def load_nav(path):
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 23:
        raise ValueError(f"{path} has {data.shape[1]} columns, expected at least 23")

    time = data[:, 2]
    rel_time = time - time[0]
    roll = data[:, 5]
    pitch = data[:, 6]
    yaw = data[:, 7]
    yaw_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(yaw)))

    ekf_lat = data[:, 11]
    ekf_lon = data[:, 12]
    ekf_alt = data[:, 13]
    gnss_lat = data[:, 20]
    gnss_lon = data[:, 21]
    gnss_alt = data[:, 22]

    lat0 = np.deg2rad(np.nanmedian(gnss_lat))
    north_err = (ekf_lat - gnss_lat) * math.pi / 180.0 * R_EARTH
    east_err = (ekf_lon - gnss_lon) * math.pi / 180.0 * R_EARTH * math.cos(lat0)
    up_err = ekf_alt - gnss_alt
    horiz_err = np.hypot(north_err, east_err)

    return {
        "path": Path(path),
        "data": data,
        "time": time,
        "rel_time": rel_time,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "yaw_unwrapped": yaw_unwrapped,
        "north_err": north_err,
        "east_err": east_err,
        "up_err": up_err,
        "horiz_err": horiz_err,
    }


def rms(x):
    x = np.asarray(x)
    return float(np.sqrt(np.mean(x * x)))


def pctl(x, q):
    return float(np.percentile(np.asarray(x), q))


def summarize(nav):
    t = nav["rel_time"]
    dt = np.diff(t)
    dt = dt[dt > 0]

    yaw_span = float(nav["yaw_unwrapped"][-1] - nav["yaw_unwrapped"][0])
    yaw_rate = np.diff(nav["yaw_unwrapped"]) / np.diff(t) if len(t) > 1 else np.array([0.0])
    yaw_rate = yaw_rate[np.isfinite(yaw_rate)]

    return {
        "file": str(nav["path"]),
        "samples": int(len(t)),
        "duration_s": float(t[-1] - t[0]) if len(t) else 0.0,
        "mean_dt_s": float(np.mean(dt)) if len(dt) else 0.0,
        "roll_min_deg": float(np.min(nav["roll"])),
        "roll_max_deg": float(np.max(nav["roll"])),
        "roll_std_deg": float(np.std(nav["roll"])),
        "pitch_min_deg": float(np.min(nav["pitch"])),
        "pitch_max_deg": float(np.max(nav["pitch"])),
        "pitch_std_deg": float(np.std(nav["pitch"])),
        "yaw_wrapped_min_deg": float(np.min(nav["yaw"])),
        "yaw_wrapped_max_deg": float(np.max(nav["yaw"])),
        "yaw_unwrapped_span_deg": yaw_span,
        "yaw_rate_rms_deg_s": rms(yaw_rate),
        "horizontal_error_rms_m": rms(nav["horiz_err"]),
        "horizontal_error_median_m": pctl(nav["horiz_err"], 50),
        "horizontal_error_95_m": pctl(nav["horiz_err"], 95),
        "horizontal_error_max_m": float(np.max(nav["horiz_err"])),
        "vertical_error_rms_m": rms(nav["up_err"]),
        "vertical_error_median_abs_m": pctl(np.abs(nav["up_err"]), 50),
        "vertical_error_95_abs_m": pctl(np.abs(nav["up_err"]), 95),
        "vertical_error_max_abs_m": float(np.max(np.abs(nav["up_err"]))),
    }


def plot_attitude(nav, out, title):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for ax, key, label in [
        (axes[0], "roll", "Roll [deg]"),
        (axes[1], "pitch", "Pitch [deg]"),
        (axes[2], "yaw", "Yaw [deg]"),
    ]:
        ax.plot(nav["rel_time"], nav[key], linewidth=1.0)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.35)
    axes[-1].set_xlabel("Time since nav start [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_yaw_unwrapped(nav, out, title):
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(nav["rel_time"], nav["yaw_unwrapped"], linewidth=1.0)
    ax.set_xlabel("Time since nav start [s]")
    ax.set_ylabel("Yaw unwrapped [deg]")
    ax.set_title(title)
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_compare(current, previous, out):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    items = [("roll", "Roll [deg]"), ("pitch", "Pitch [deg]"), ("yaw_unwrapped", "Yaw unwrapped [deg]")]
    for ax, (key, label) in zip(axes, items):
        ax.plot(previous["rel_time"], previous[key], label=previous["path"].stem, linewidth=0.9, alpha=0.85)
        ax.plot(current["rel_time"], current[key], label=current["path"].stem, linewidth=0.9, alpha=0.85)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.35)
        ax.legend(loc="best")
    axes[-1].set_xlabel("Time since nav start [s]")
    fig.suptitle("Attitude comparison")
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_error_compare(current, previous, out):
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
    axes[0].plot(previous["rel_time"], previous["horiz_err"], label=previous["path"].stem, linewidth=0.8, alpha=0.85)
    axes[0].plot(current["rel_time"], current["horiz_err"], label=current["path"].stem, linewidth=0.8, alpha=0.85)
    axes[0].set_ylabel("EKF-GNSS horizontal [m]")
    axes[0].grid(True, alpha=0.35)
    axes[0].legend(loc="best")

    axes[1].plot(previous["rel_time"], previous["up_err"], label=previous["path"].stem, linewidth=0.8, alpha=0.85)
    axes[1].plot(current["rel_time"], current["up_err"], label=current["path"].stem, linewidth=0.8, alpha=0.85)
    axes[1].set_xlabel("Time since nav start [s]")
    axes[1].set_ylabel("EKF-GNSS height [m]")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend(loc="best")

    fig.suptitle("Position residual comparison")
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--current", required=True)
    parser.add_argument("--previous", required=True)
    parser.add_argument("--outdir", default="/home/whysad/实验数据处理/plot_outputs")
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    current = load_nav(args.current)
    previous = load_nav(args.previous)
    current_id = current["path"].stem.replace("nav", "")
    previous_id = previous["path"].stem.replace("nav", "")

    outputs = {
        "current_attitude": str(outdir / f"{current_id}_attitude.png"),
        "current_yaw_unwrapped": str(outdir / f"{current_id}_attitude_yaw_unwrapped.png"),
        "attitude_compare": str(outdir / f"{previous_id}_vs_{current_id}_attitude_compare.png"),
        "position_residual_compare": str(outdir / f"{previous_id}_vs_{current_id}_position_residual_compare.png"),
        "summary": str(outdir / f"{previous_id}_vs_{current_id}_summary.json"),
    }

    plot_attitude(current, outputs["current_attitude"], f"{current_id} attitude")
    plot_yaw_unwrapped(current, outputs["current_yaw_unwrapped"], f"{current_id} yaw unwrapped")
    plot_compare(current, previous, outputs["attitude_compare"])
    plot_error_compare(current, previous, outputs["position_residual_compare"])

    summary = {
        "previous": summarize(previous),
        "current": summarize(current),
        "notes": [
            "Attitude columns are nav.txt columns 5, 6, 7: roll, pitch, yaw in degrees.",
            "Position residual uses EKF lat/lon/height columns 11-13 minus GNSS columns 20-22.",
            "GNSS residual is a consistency check, not an independent ground-truth accuracy evaluation.",
        ],
        "outputs": outputs,
    }
    Path(outputs["summary"]).write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
