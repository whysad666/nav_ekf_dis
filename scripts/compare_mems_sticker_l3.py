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
    if data.shape[1] < 14:
        raise ValueError(f"{path}: expected at least 14 columns")
    return data


def wrap_deg(values):
    return (np.asarray(values) + 180.0) % 360.0 - 180.0


def interp_linear(reference, times, columns):
    return np.column_stack(
        [np.interp(times, reference[:, 2], reference[:, col]) for col in columns]
    )


def interp_attitude(reference, times):
    roll_pitch = interp_linear(reference, times, (5, 6))
    yaw_unwrapped = np.unwrap(np.deg2rad(reference[:, 7]))
    yaw = np.rad2deg(np.interp(times, reference[:, 2], yaw_unwrapped))
    return np.column_stack((roll_pitch, wrap_deg(yaw)))


def position_delta_m(source_position, reference_position):
    lat0 = math.radians(float(np.mean(reference_position[:, 0])))
    north = (
        (source_position[:, 0] - reference_position[:, 0])
        * math.pi
        / 180.0
        * R_EARTH
    )
    east = (
        (source_position[:, 1] - reference_position[:, 1])
        * math.pi
        / 180.0
        * R_EARTH
        * math.cos(lat0)
    )
    up = source_position[:, 2] - reference_position[:, 2]
    return np.column_stack((north, east, up))


def rms(values, axis=None):
    return np.sqrt(np.mean(np.asarray(values) ** 2, axis=axis))


def position_stats(errors):
    horizontal = np.linalg.norm(errors[:, :2], axis=1)
    return {
        "horizontal_rms_m": float(rms(horizontal)),
        "horizontal_median_m": float(np.median(horizontal)),
        "horizontal_p95_m": float(np.percentile(horizontal, 95)),
        "horizontal_max_m": float(np.max(horizontal)),
        "vertical_rms_m": float(rms(errors[:, 2])),
        "vertical_median_m": float(np.median(errors[:, 2])),
    }


def calculate_metrics(current, reference, common_start, common_end, takeoff_time):
    keep = (current[:, 2] >= common_start) & (current[:, 2] <= common_end)
    current = current[keep]
    times = current[:, 2]
    reference_attitude = interp_attitude(reference, times)
    reference_position = interp_linear(reference, times, (11, 12, 13))

    attitude_error = current[:, 5:8] - reference_attitude
    attitude_error[:, 2] = wrap_deg(attitude_error[:, 2])
    position_error = position_delta_m(current[:, 11:14], reference_position)

    initial = times < common_start + 10.0
    flight = times >= takeoff_time
    initial_position_offset = np.median(position_error[initial], axis=0)
    corrected_position_error = position_error - initial_position_offset

    metrics = {
        "samples": int(len(current)),
        "first_time": float(times[0]),
        "last_time": float(times[-1]),
        "common_start_10s_attitude_error_median_deg": np.median(
            attitude_error[initial], axis=0
        ).tolist(),
        "flight_attitude_error_deg": {
            "rms": rms(attitude_error[flight], axis=0).tolist(),
            "p95_absolute": np.percentile(
                np.abs(attitude_error[flight]), 95, axis=0
            ).tolist(),
            "maximum_absolute": np.max(
                np.abs(attitude_error[flight]), axis=0
            ).tolist(),
        },
        "initial_neu_offset_m": initial_position_offset.tolist(),
        "position_error_common_raw": position_stats(position_error),
        "position_error_flight_raw": position_stats(position_error[flight]),
        "position_error_flight_after_initial_offset_removal": position_stats(
            corrected_position_error[flight]
        ),
    }
    return current, attitude_error, position_error, corrected_position_error, metrics


def plot_trajectory(reference, mems, sticker, common_start, common_end, output):
    keep = (reference[:, 2] >= common_start) & (reference[:, 2] <= common_end)
    ref = reference[keep]
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(ref[:, 12], ref[:, 11], label="L3 high-grade INS", linewidth=1.8)
    ax.plot(mems[:, 12], mems[:, 11], label="L3 MEMS EKF", linewidth=1.0)
    ax.plot(
        sticker[:, 12],
        sticker[:, 11],
        label="small-system sticker IMU EKF",
        linewidth=1.0,
    )
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title("Trajectory comparison on the common interval")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_attitude(reference, mems, sticker, common_start, common_end, output):
    keep = (reference[:, 2] >= common_start) & (reference[:, 2] <= common_end)
    ref = reference[keep]
    labels = ("Roll [deg]", "Pitch [deg]", "Yaw [deg]")
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for index, ax in enumerate(axes):
        ax.plot(
            ref[:, 2] - common_start,
            ref[:, 5 + index],
            label="L3 high-grade INS",
            linewidth=1.4,
        )
        ax.plot(
            mems[:, 2] - common_start,
            mems[:, 5 + index],
            label="L3 MEMS EKF",
            linewidth=0.9,
        )
        ax.plot(
            sticker[:, 2] - common_start,
            sticker[:, 5 + index],
            label="small-system sticker IMU EKF",
            linewidth=0.9,
            alpha=0.85,
        )
        ax.set_ylabel(labels[index])
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time since common interval start [s]")
    fig.suptitle("Attitude comparison on the common interval")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_errors(
    mems,
    mems_error,
    mems_position_error,
    sticker,
    sticker_error,
    sticker_position_error,
    common_start,
    output,
):
    labels = ("Roll error [deg]", "Pitch error [deg]", "Yaw error [deg]")
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    for index in range(3):
        axes[index].plot(
            mems[:, 2] - common_start,
            mems_error[:, index],
            label="L3 MEMS EKF",
            linewidth=0.9,
        )
        axes[index].plot(
            sticker[:, 2] - common_start,
            sticker_error[:, index],
            label="small-system sticker IMU EKF",
            linewidth=0.9,
            alpha=0.85,
        )
        axes[index].set_ylabel(labels[index])
        axes[index].grid(True, alpha=0.3)
    axes[3].plot(
        mems[:, 2] - common_start,
        np.linalg.norm(mems_position_error[:, :2], axis=1),
        label="L3 MEMS EKF",
        linewidth=0.9,
    )
    axes[3].plot(
        sticker[:, 2] - common_start,
        np.linalg.norm(sticker_position_error[:, :2], axis=1),
        label="small-system sticker IMU EKF",
        linewidth=0.9,
        alpha=0.85,
    )
    axes[3].set_ylabel("Horizontal error [m]")
    axes[3].set_xlabel("Time since common interval start [s]")
    axes[3].grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[2].set_xlabel("Time since common interval start [s]")
    fig.suptitle("Attitude errors relative to L3 high-grade INS")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mems", required=True)
    parser.add_argument("--sticker", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--takeoff-time", type=float, default=1780479430.498)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    mems_all = load(args.mems)
    sticker_all = load(args.sticker)
    reference = load(args.reference)
    common_start = max(
        mems_all[0, 2], sticker_all[0, 2], reference[0, 2]
    )
    common_end = min(
        mems_all[-1, 2], sticker_all[-1, 2], reference[-1, 2]
    )
    if common_end <= common_start:
        raise RuntimeError("navigation files do not share a common time interval")

    mems, mems_att_error, mems_pos_error, _, mems_metrics = calculate_metrics(
        mems_all, reference, common_start, common_end, args.takeoff_time
    )
    sticker, sticker_att_error, sticker_pos_error, _, sticker_metrics = calculate_metrics(
        sticker_all, reference, common_start, common_end, args.takeoff_time
    )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "trajectory": str(output_dir / "mems_sticker_l3_trajectory.png"),
        "attitude": str(output_dir / "mems_sticker_l3_attitude.png"),
        "attitude_errors": str(output_dir / "mems_sticker_l3_attitude_errors.png"),
        "summary": str(output_dir / "mems_sticker_l3_summary.json"),
    }

    plot_trajectory(reference, mems, sticker, common_start, common_end, outputs["trajectory"])
    plot_attitude(reference, mems, sticker, common_start, common_end, outputs["attitude"])
    plot_errors(
        mems,
        mems_att_error,
        mems_pos_error,
        sticker,
        sticker_att_error,
        sticker_pos_error,
        common_start,
        outputs["attitude_errors"],
    )

    summary = {
        "files": {
            "l3_mems": args.mems,
            "small_system_sticker": args.sticker,
            "l3_high_grade_reference": args.reference,
        },
        "common_interval": {
            "start": float(common_start),
            "end": float(common_end),
            "duration_s": float(common_end - common_start),
            "takeoff_time": args.takeoff_time,
        },
        "l3_mems": mems_metrics,
        "small_system_sticker": sticker_metrics,
        "outputs": outputs,
        "notes": [
            "All metrics use only the interval shared by all three navigation files.",
            "Attitude errors are relative to the L3 high-grade INS; yaw is interpolated and differenced circularly.",
            "Offset-removed position errors subtract the median NEU error over the first 10 seconds of the common interval.",
        ],
    }
    Path(outputs["summary"]).write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
