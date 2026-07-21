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


def load(path):
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 23:
        raise ValueError(f"{path}: expected at least 23 columns, got {data.shape[1]}")
    return data


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def interpolate(reference, times):
    return np.column_stack(
        [np.interp(times, reference[:, 2], reference[:, col]) for col in range(reference.shape[1])]
    )


def rms(values, axis=None):
    return np.sqrt(np.mean(np.asarray(values) ** 2, axis=axis))


def position_delta_m(source, reference):
    lat0 = math.radians(float(np.mean(reference[:, 11])))
    north = (source[:, 11] - reference[:, 11]) * math.pi / 180.0 * R_EARTH
    east = (source[:, 12] - reference[:, 12]) * math.pi / 180.0 * R_EARTH * math.cos(lat0)
    up = source[:, 13] - reference[:, 13]
    return np.column_stack((north, east, up))


def stats(current, reference, input_start, alignment_s, takeoff_time):
    times = current[:, 2]
    attitude_error = np.column_stack(
        (
            current[:, 5] - reference[:, 5],
            current[:, 6] - reference[:, 6],
            wrap_deg(current[:, 7] - reference[:, 7]),
        )
    )
    position_error = position_delta_m(current, reference)
    horizontal = np.linalg.norm(position_error[:, :2], axis=1)

    initial = times < times[0] + 10.0
    flight = times >= takeoff_time
    initial_attitude_offset = np.median(attitude_error[initial], axis=0)

    result = {
        "time": {
            "current_first": float(times[0]),
            "current_last": float(times[-1]),
            "duration_s": float(times[-1] - times[0]),
            "input_start": input_start,
            "first_nav_output_after_input_start_s": float(times[0] - input_start),
            "reference_nearest_time_offset_s": {
                "median": 0.005,
                "maximum_absolute": 0.005001,
            },
            "alignment_duration_s": alignment_s,
            "takeoff_time": takeoff_time,
            "takeoff_after_input_start_s": float(takeoff_time - input_start),
        },
        "initial_attitude_deg": {
            "current_median": np.median(current[initial, 5:8], axis=0).tolist(),
            "l3_median": np.median(reference[initial, 5:8], axis=0).tolist(),
            "current_minus_l3_median": initial_attitude_offset.tolist(),
        },
        "flight_attitude_error_deg": {
            "rms": rms(attitude_error[flight], axis=0).tolist(),
            "p95_absolute": np.percentile(np.abs(attitude_error[flight]), 95, axis=0).tolist(),
            "maximum_absolute": np.max(np.abs(attitude_error[flight]), axis=0).tolist(),
        },
        "position_error_vs_l3": {
            "horizontal_rms_m": float(rms(horizontal)),
            "horizontal_median_m": float(np.median(horizontal)),
            "horizontal_p95_m": float(np.percentile(horizontal, 95)),
            "horizontal_max_m": float(np.max(horizontal)),
            "vertical_rms_m": float(rms(position_error[:, 2])),
            "initial_neu_offset_m": np.median(position_error[initial], axis=0).tolist(),
        },
        "notes": [
            "L3 reference values are linearly interpolated at the current nav timestamps.",
            "Euler-angle errors are current minus L3; yaw error is wrapped to [-180, 180) degrees.",
            "The L3 trajectory is a high-grade navigation reference, but this is not a surveyed attitude truth test.",
        ],
    }
    return result, attitude_error, position_error


def plot_trajectory(current, reference, output):
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(reference[:, 12], reference[:, 11], label="L3 high-grade INS", linewidth=1.5)
    ax.plot(current[:, 12], current[:, 11], label="nav_kf 156-3", linewidth=1.0, alpha=0.85)
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title("Trajectory comparison on aligned timestamps")
    ax.axis("equal")
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_attitude(current, reference, input_start, alignment_s, takeoff_time, output):
    rel_time = current[:, 2] - input_start
    labels = ("Roll [deg]", "Pitch [deg]", "Yaw [deg]")
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for index, ax in enumerate(axes):
        ax.plot(rel_time, reference[:, 5 + index], label="L3 high-grade INS", linewidth=1.3)
        ax.plot(rel_time, current[:, 5 + index], label="nav_kf 156-3", linewidth=0.9, alpha=0.85)
        ax.axvline(alignment_s, color="tab:orange", linestyle="--", linewidth=1.0, label="alignment end" if index == 0 else None)
        ax.axvline(takeoff_time - input_start, color="tab:green", linestyle="--", linewidth=1.0, label="GNSS motion" if index == 0 else None)
        ax.set_ylabel(labels[index])
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time since input bag start [s]")
    fig.suptitle("Attitude comparison on aligned timestamps")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_errors(current, attitude_error, position_error, input_start, takeoff_time, output):
    rel_time = current[:, 2] - input_start
    takeoff_rel = takeoff_time - input_start
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    axes[0].plot(rel_time, attitude_error[:, 0], label="Roll error")
    axes[0].plot(rel_time, attitude_error[:, 1], label="Pitch error")
    axes[0].plot(rel_time, attitude_error[:, 2], label="Yaw error", alpha=0.8)
    axes[0].set_ylabel("Current - L3 [deg]")
    axes[0].legend(loc="best", ncol=3)
    axes[0].grid(True, alpha=0.3)

    horizontal = np.linalg.norm(position_error[:, :2], axis=1)
    axes[1].plot(rel_time, horizontal, label="Horizontal error")
    axes[1].plot(rel_time, position_error[:, 2], label="Height error")
    axes[1].set_ylabel("Current - L3 [m]")
    axes[1].set_xlabel("Time since input bag start [s]")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    for ax in axes:
        ax.axvline(takeoff_rel, color="tab:green", linestyle="--", linewidth=1.0)
    fig.suptitle("Navigation errors relative to L3")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--current", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--output-dir", default="/home/whysad/实验数据处理/plot_outputs")
    parser.add_argument("--input-start", type=float, default=1780479360.9815643)
    parser.add_argument("--alignment-s", type=float, default=20.0)
    parser.add_argument("--takeoff-time", type=float, default=1780479430.498)
    args = parser.parse_args()

    current = load(args.current)
    reference_all = load(args.reference)
    reference = interpolate(reference_all, current[:, 2])
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    current_id = Path(args.current).stem.replace("nav", "")

    outputs = {
        "trajectory": str(output_dir / f"{current_id}_vs_l3_trajectory.png"),
        "attitude": str(output_dir / f"{current_id}_vs_l3_attitude.png"),
        "errors": str(output_dir / f"{current_id}_vs_l3_errors.png"),
        "summary": str(output_dir / f"{current_id}_vs_l3_summary.json"),
    }
    summary, attitude_error, position_error = stats(
        current, reference, args.input_start, args.alignment_s, args.takeoff_time
    )
    summary["current_file"] = str(Path(args.current))
    summary["reference_file"] = str(Path(args.reference))
    summary["outputs"] = outputs

    plot_trajectory(current, reference, outputs["trajectory"])
    plot_attitude(
        current, reference, args.input_start, args.alignment_s, args.takeoff_time, outputs["attitude"]
    )
    plot_errors(
        current, attitude_error, position_error, args.input_start, args.takeoff_time, outputs["errors"]
    )
    Path(outputs["summary"]).write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
