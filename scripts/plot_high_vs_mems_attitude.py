#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from compare_mems_sticker_l3 import (
    interp_attitude,
    interp_linear,
    load,
    position_delta_m,
    position_stats,
    rms,
    wrap_deg,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--high", required=True)
    parser.add_argument("--mems", required=True)
    parser.add_argument("--takeoff-time", type=float, default=1780479430.498)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    high = load(args.high)
    mems_all = load(args.mems)
    common_start = max(high[0, 2], mems_all[0, 2])
    common_end = min(high[-1, 2], mems_all[-1, 2])
    if common_end <= common_start:
        raise RuntimeError("inputs do not share a common time interval")

    high_keep = (high[:, 2] >= common_start) & (high[:, 2] <= common_end)
    mems_keep = (mems_all[:, 2] >= common_start) & (mems_all[:, 2] <= common_end)
    high_plot = high[high_keep]
    mems = mems_all[mems_keep]

    high_at_mems = interp_attitude(high, mems[:, 2])
    errors = mems[:, 5:8] - high_at_mems
    errors[:, 2] = wrap_deg(errors[:, 2])
    high_position_at_mems = interp_linear(high, mems[:, 2], (11, 12, 13))
    position_errors = position_delta_m(mems[:, 11:14], high_position_at_mems)
    flight = mems[:, 2] >= args.takeoff_time
    settled_flight = mems[:, 2] >= args.takeoff_time + 20.0
    initial = mems[:, 2] < common_start + 10.0

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    attitude_path = output_dir / "high_vs_mems_attitude.png"
    errors_path = output_dir / "high_vs_mems_attitude_errors.png"
    trajectory_path = output_dir / "high_vs_mems_trajectory.png"
    position_errors_path = output_dir / "high_vs_mems_position_errors.png"
    summary_path = output_dir / "high_vs_mems_attitude_summary.json"

    labels = ("Roll [deg]", "Pitch [deg]", "Yaw [deg]")
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for index, ax in enumerate(axes):
        ax.plot(
            high_plot[:, 2] - common_start,
            high_plot[:, 5 + index],
            label="L3 high-grade IMU EKF",
            linewidth=1.4,
        )
        ax.plot(
            mems[:, 2] - common_start,
            mems[:, 5 + index],
            label="L3 MEMS IMU EKF",
            linewidth=0.9,
            alpha=0.9,
        )
        ax.axvline(
            args.takeoff_time - common_start,
            color="tab:green",
            linestyle="--",
            linewidth=1.0,
            label="takeoff" if index == 0 else None,
        )
        ax.set_ylabel(labels[index])
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time since common interval start [s]")
    fig.suptitle("High-grade IMU and MEMS IMU attitude comparison")
    fig.tight_layout()
    fig.savefig(attitude_path, dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for index, ax in enumerate(axes):
        ax.plot(mems[:, 2] - common_start, errors[:, index], linewidth=1.0)
        ax.axvline(
            args.takeoff_time - common_start,
            color="tab:green",
            linestyle="--",
            linewidth=1.0,
        )
        ax.axhline(0.0, color="black", linewidth=0.7, alpha=0.6)
        ax.set_ylabel(f"{labels[index].split()[0]} error [deg]")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time since common interval start [s]")
    fig.suptitle("MEMS attitude error relative to high-grade IMU")
    fig.tight_layout()
    fig.savefig(errors_path, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(
        high_plot[:, 12],
        high_plot[:, 11],
        label="L3 high-grade IMU EKF",
        linewidth=1.6,
    )
    ax.plot(
        mems[:, 12],
        mems[:, 11],
        label="L3 MEMS IMU EKF",
        linewidth=1.0,
        alpha=0.9,
    )
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title("High-grade IMU and MEMS IMU trajectory comparison")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(trajectory_path, dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for index, label in enumerate(("North error [m]", "East error [m]", "Up error [m]")):
        axes[index].plot(
            mems[:, 2] - common_start,
            position_errors[:, index],
            linewidth=1.0,
        )
        axes[index].axvline(
            args.takeoff_time - common_start,
            color="tab:green",
            linestyle="--",
            linewidth=1.0,
        )
        axes[index].axhline(0.0, color="black", linewidth=0.7, alpha=0.6)
        axes[index].set_ylabel(label)
        axes[index].grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time since common interval start [s]")
    fig.suptitle("MEMS position error relative to high-grade IMU")
    fig.tight_layout()
    fig.savefig(position_errors_path, dpi=180)
    plt.close(fig)

    summary = {
        "files": {"high_grade": args.high, "mems": args.mems},
        "common_interval": {
            "start": float(common_start),
            "end": float(common_end),
            "duration_s": float(common_end - common_start),
            "takeoff_time": args.takeoff_time,
            "takeoff_after_common_start_s": float(args.takeoff_time - common_start),
        },
        "common_start_10s_error_median_deg": np.median(
            errors[initial], axis=0
        ).tolist(),
        "flight_error_deg": {
            "rms": rms(errors[flight], axis=0).tolist(),
            "p95_absolute": np.percentile(
                np.abs(errors[flight]), 95, axis=0
            ).tolist(),
            "maximum_absolute": np.max(
                np.abs(errors[flight]), axis=0
            ).tolist(),
        },
        "settled_flight_error_after_20s_deg": {
            "median": np.median(errors[settled_flight], axis=0).tolist(),
            "rms": rms(errors[settled_flight], axis=0).tolist(),
            "p95_absolute": np.percentile(
                np.abs(errors[settled_flight]), 95, axis=0
            ).tolist(),
        },
        "position_error_m": {
            "initial_neu_median": np.median(
                position_errors[initial], axis=0
            ).tolist(),
            "common": position_stats(position_errors),
            "flight": position_stats(position_errors[flight]),
            "settled_flight_after_20s": position_stats(
                position_errors[settled_flight]
            ),
            "flight_neu_rms": rms(position_errors[flight], axis=0).tolist(),
            "flight_neu_median": np.median(
                position_errors[flight], axis=0
            ).tolist(),
        },
        "outputs": {
            "attitude": str(attitude_path),
            "errors": str(errors_path),
            "trajectory": str(trajectory_path),
            "position_errors": str(position_errors_path),
            "summary": str(summary_path),
        },
        "notes": [
            "High-grade attitude is circularly interpolated at MEMS timestamps.",
            "Yaw error is wrapped to [-180, 180) degrees.",
            "This is relative consistency, not an independent surveyed truth test.",
        ],
    }
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
