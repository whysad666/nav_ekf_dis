#!/usr/bin/env python3
from pathlib import Path
import json
import math

import numpy as np
import rosbag
import rospy


IN_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_nav_kf_input.bag")
OUT_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_nav_kf_input_timefixed.bag")
OUT_META = OUT_BAG.with_suffix(".json")

IMU_TOPIC = "/zh_origin"
SAMPLE_SYNC_TOPICS = {"/mag_origin"}
FIXED_RATE_HZ = 200.0


def stamp_to_sec(stamp):
    return stamp.to_sec()


def build_imu_time_map():
    original_times = []
    fixed_times = []
    fixed_dt = 1.0 / FIXED_RATE_HZ

    with rosbag.Bag(str(IN_BAG)) as bag:
        for idx, (_, msg, stamp) in enumerate(bag.read_messages(topics=[IMU_TOPIC])):
            data = list(msg.data)
            original_time = float(data[0]) if data else stamp_to_sec(stamp)
            fixed_time = original_times[0] + idx * fixed_dt if original_times else original_time
            original_times.append(original_time)
            fixed_times.append(fixed_time)

    if not original_times:
        raise RuntimeError(f"No {IMU_TOPIC} messages found in {IN_BAG}")

    original = np.asarray(original_times, dtype=float)
    fixed = np.asarray(fixed_times, dtype=float)
    if np.any(np.diff(original) < 0):
        raise RuntimeError(f"{IMU_TOPIC} time is not monotonic")
    return original, fixed


def map_time(t, original, fixed):
    return float(np.interp(float(t), original, fixed))


def is_time_like(value, stamp):
    return math.isfinite(value) and abs(value - stamp) < 2.0


def rewrite_bag(original, fixed):
    counts = {}
    max_abs_stamp_shift = 0.0

    with rosbag.Bag(str(IN_BAG)) as inbag, rosbag.Bag(str(OUT_BAG), "w") as outbag:
        for topic, msg, stamp in inbag.read_messages():
            old_stamp = stamp_to_sec(stamp)

            if topic == IMU_TOPIC:
                data = list(msg.data)
                idx = counts.get(topic, 0)
                new_time = float(fixed[idx])
                data[0] = new_time
                data[1] = idx / FIXED_RATE_HZ
                msg.data = data
            elif topic in SAMPLE_SYNC_TOPICS and counts.get(topic, 0) < len(fixed):
                data = list(msg.data)
                idx = counts.get(topic, 0)
                new_time = float(fixed[idx])
                if data and is_time_like(float(data[0]), old_stamp):
                    data[0] = new_time
                    msg.data = data
            else:
                data = list(msg.data) if hasattr(msg, "data") else None
                old_time = old_stamp
                if data and is_time_like(float(data[0]), old_stamp):
                    old_time = float(data[0])
                new_time = map_time(old_time, original, fixed)
                if data and is_time_like(float(data[0]), old_stamp):
                    data[0] = new_time
                    msg.data = data

            outbag.write(topic, msg, rospy.Time.from_sec(new_time))
            counts[topic] = counts.get(topic, 0) + 1
            max_abs_stamp_shift = max(max_abs_stamp_shift, abs(new_time - old_stamp))

    return counts, max_abs_stamp_shift


def write_meta(original, fixed, counts, max_abs_stamp_shift):
    meta = {
        "input_bag": str(IN_BAG),
        "output_bag": str(OUT_BAG),
        "imu_topic": IMU_TOPIC,
        "fixed_rate_hz": FIXED_RATE_HZ,
        "imu_messages": int(len(original)),
        "original_imu_duration_s": float(original[-1] - original[0]),
        "fixed_imu_duration_s": float(fixed[-1] - fixed[0]),
        "removed_cumulative_time_error_s": float((original[-1] - original[0]) - (fixed[-1] - fixed[0])),
        "max_abs_stamp_shift_s": float(max_abs_stamp_shift),
        "topic_counts": counts,
        "notes": [
            "All topic timestamps are mapped onto the continuous 200 Hz IMU count timebase.",
            "For time-like Float64MultiArray data[0], data[0] is rewritten to the mapped timestamp.",
            "The IMU data[1] field is rewritten as sample_index / 200.",
            "IMU dtheta/dvel values are unchanged.",
        ],
    }
    OUT_META.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")


def main():
    print(f"Input : {IN_BAG}")
    print(f"Output: {OUT_BAG}")
    original, fixed = build_imu_time_map()
    print(f"IMU original duration: {original[-1] - original[0]:.6f} s")
    print(f"IMU fixed duration   : {fixed[-1] - fixed[0]:.6f} s")
    print(f"Time error removed   : {(original[-1] - original[0]) - (fixed[-1] - fixed[0]):.6f} s")
    counts, max_abs_stamp_shift = rewrite_bag(original, fixed)
    write_meta(original, fixed, counts, max_abs_stamp_shift)
    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(f"Max timestamp shift: {max_abs_stamp_shift:.6f} s")
    for topic in sorted(counts):
        print(f"{topic}: {counts[topic]}")


if __name__ == "__main__":
    main()
