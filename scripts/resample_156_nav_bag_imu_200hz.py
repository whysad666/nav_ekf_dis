#!/usr/bin/env python3
from pathlib import Path
import json

import numpy as np
import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


IN_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_nav_kf_input.bag")
OUT_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_nav_kf_input_resampled_200hz_trapz.bag")
OUT_META = OUT_BAG.with_suffix(".json")

IMU_TOPIC = "/zh_origin"
MAG_TOPIC = "/mag_origin"
RATE_HZ = 200.0
DT = 1.0 / RATE_HZ


def read_array_topic(topic):
    times = []
    values = []
    with rosbag.Bag(str(IN_BAG)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            data = np.asarray(msg.data, dtype=float)
            times.append(float(data[0]) if len(data) else stamp.to_sec())
            values.append(data)
    if not times:
        return np.empty(0), np.empty((0, 0))
    return np.asarray(times, dtype=float), np.vstack(values)


def require_strictly_increasing(times, topic):
    bad = np.where(np.diff(times) <= 0)[0]
    if len(bad):
        raise RuntimeError(f"{topic} has non-increasing timestamps near index {bad[0]}")


def interp_columns(new_times, old_times, old_values):
    cols = []
    for col in range(old_values.shape[1]):
        cols.append(np.interp(new_times, old_times, old_values[:, col]))
    return np.vstack(cols).T


def make_msg(data):
    msg = Float64MultiArray()
    msg.data = [float(x) for x in data]
    return msg


def write_resampled_bag(sample_times, imu_increments, mag_values):
    counts = {}
    imu_idx = 0
    mag_idx = 0

    with rosbag.Bag(str(IN_BAG)) as inbag, rosbag.Bag(str(OUT_BAG), "w") as outbag:
        # Write non-resampled topics in original order, and inject resampled IMU/MAG
        # before each original message whose timestamp is later than the next sample.
        for topic, msg, stamp in inbag.read_messages():
            old_time = stamp.to_sec()
            while imu_idx < len(sample_times) and sample_times[imu_idx] <= old_time:
                t = float(sample_times[imu_idx])
                out = make_msg([
                    t,
                    imu_idx * DT,
                    imu_increments[imu_idx, 0],
                    imu_increments[imu_idx, 1],
                    imu_increments[imu_idx, 2],
                    imu_increments[imu_idx, 3],
                    imu_increments[imu_idx, 4],
                    imu_increments[imu_idx, 5],
                ])
                outbag.write(IMU_TOPIC, out, rospy.Time.from_sec(t))
                counts[IMU_TOPIC] = counts.get(IMU_TOPIC, 0) + 1
                imu_idx += 1

            while mag_idx < len(sample_times) and sample_times[mag_idx] <= old_time:
                t = float(sample_times[mag_idx])
                out = make_msg([t, mag_values[mag_idx, 0], mag_values[mag_idx, 1], mag_values[mag_idx, 2]])
                outbag.write(MAG_TOPIC, out, rospy.Time.from_sec(t))
                counts[MAG_TOPIC] = counts.get(MAG_TOPIC, 0) + 1
                mag_idx += 1

            if topic in {IMU_TOPIC, MAG_TOPIC}:
                continue

            outbag.write(topic, msg, stamp)
            counts[topic] = counts.get(topic, 0) + 1

        while imu_idx < len(sample_times):
            t = float(sample_times[imu_idx])
            out = make_msg([
                t,
                imu_idx * DT,
                imu_increments[imu_idx, 0],
                imu_increments[imu_idx, 1],
                imu_increments[imu_idx, 2],
                imu_increments[imu_idx, 3],
                imu_increments[imu_idx, 4],
                imu_increments[imu_idx, 5],
            ])
            outbag.write(IMU_TOPIC, out, rospy.Time.from_sec(t))
            counts[IMU_TOPIC] = counts.get(IMU_TOPIC, 0) + 1
            imu_idx += 1

        while mag_idx < len(sample_times):
            t = float(sample_times[mag_idx])
            out = make_msg([t, mag_values[mag_idx, 0], mag_values[mag_idx, 1], mag_values[mag_idx, 2]])
            outbag.write(MAG_TOPIC, out, rospy.Time.from_sec(t))
            counts[MAG_TOPIC] = counts.get(MAG_TOPIC, 0) + 1
            mag_idx += 1

    return counts


def main():
    print(f"Input : {IN_BAG}")
    print(f"Output: {OUT_BAG}")

    imu_times, imu_data = read_array_topic(IMU_TOPIC)
    mag_times, mag_data = read_array_topic(MAG_TOPIC)
    require_strictly_increasing(imu_times, IMU_TOPIC)
    require_strictly_increasing(mag_times, MAG_TOPIC)

    # The prepared input stores increments made with fixed 0.005 s.
    # Recover angular-rate/specific-force samples, resample those rates on the
    # original experiment time axis, then integrate each fixed 0.005 s interval
    # with the trapezoidal rule to generate new increments.
    imu_rates = np.column_stack((imu_data[:, 2:5] / DT, imu_data[:, 5:8] / DT))

    interval_count = int(np.floor((imu_times[-1] - imu_times[0]) * RATE_HZ))
    sample_times = imu_times[0] + np.arange(interval_count, dtype=float) * DT
    edge_times = imu_times[0] + np.arange(interval_count + 1, dtype=float) * DT

    edge_rates = interp_columns(edge_times, imu_times, imu_rates)
    resampled_imu_increments = 0.5 * (edge_rates[:-1] + edge_rates[1:]) * DT
    resampled_mag = interp_columns(sample_times, mag_times, mag_data[:, 1:4])

    counts = write_resampled_bag(sample_times, resampled_imu_increments, resampled_mag)

    original_gaps = np.diff(imu_times)
    meta = {
        "input_bag": str(IN_BAG),
        "output_bag": str(OUT_BAG),
        "rate_hz": RATE_HZ,
        "original_imu_messages": int(len(imu_times)),
        "resampled_imu_messages": int(len(sample_times)),
        "original_imu_duration_s": float(imu_times[-1] - imu_times[0]),
        "resampled_imu_duration_s": float(sample_times[-1] - sample_times[0]),
        "original_imu_gap_median_s": float(np.median(original_gaps)),
        "original_imu_gap_max_s": float(np.max(original_gaps)),
        "original_imu_gap_gt_20ms": int(np.sum(original_gaps > 0.02)),
        "topic_counts": counts,
        "notes": [
            "This is the physically consistent fixed-200Hz input for nav_kf.",
            "IMU angular-rate and acceleration samples are recovered from the prepared increments and linearly interpolated on the original experiment time axis.",
            "New dtheta/dvel are generated by trapezoidal integration over each fixed 0.005 s interval.",
            "GNSS and range topics keep their original experiment timestamps.",
            "MAG is resampled to the same 200Hz grid as IMU for initial yaw alignment.",
        ],
    }
    OUT_META.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(f"Original IMU messages : {len(imu_times)}")
    print(f"Resampled IMU messages: {len(sample_times)}")
    print(f"Original duration     : {imu_times[-1] - imu_times[0]:.6f} s")
    print(f"Resampled duration    : {sample_times[-1] - sample_times[0]:.6f} s")
    print(f"Original max gap      : {np.max(original_gaps):.6f} s")
    for topic in sorted(counts):
        print(f"{topic}: {counts[topic]}")


if __name__ == "__main__":
    main()
