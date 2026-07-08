#!/usr/bin/env python3
import json
from pathlib import Path

import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


PREPARED_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_nav_kf_input_resampled_200hz_trapz.bag")
HIGH_IMU_BAG = Path("/media/whysad/Lenovo1/260614铜官窑实验/l型/0614_2.bag")
OUT_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_lidar_imu_prepared_obs_51s.bag")
OUT_META = OUT_BAG.with_suffix(".json")

IMU_TOPIC = "/zh_origin"
MAG_TOPIC = "/mag_origin"
GNSS_TOPIC = "/gnss_origin"
RANGE_TOPIC = "/range"
HIGH_MAG_SOURCE_TOPIC = "/L3/imu_origin"

IMU_RATE_HZ = 200.0
DT = 1.0 / IMU_RATE_HZ


def first_time(bag_path, topic):
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            data = list(getattr(msg, "data", []))
            if data:
                return float(data[0])
            return stamp.to_sec()
    raise RuntimeError(f"No messages on {topic} in {bag_path}")


def last_time(bag_path, topic):
    last = None
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            data = list(getattr(msg, "data", []))
            last = float(data[0]) if data else stamp.to_sec()
    if last is None:
        raise RuntimeError(f"No messages on {topic} in {bag_path}")
    return last


def shifted_time(src_time, src_start, dst_start):
    return dst_start + (src_time - src_start)


def make_msg(values):
    msg = Float64MultiArray()
    msg.data = [float(v) for v in values]
    return msg


def write_high_imu(outbag, src_start, dst_start, duration):
    count = 0
    with rosbag.Bag(str(HIGH_IMU_BAG)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[IMU_TOPIC]):
            data = list(msg.data)
            if len(data) < 8:
                continue
            src_t = float(data[0])
            rel = src_t - src_start
            if rel < 0:
                continue
            if rel > duration:
                break
            dst_t = shifted_time(src_t, src_start, dst_start)
            # nav_kf only consumes columns 0..7. Reset steady time so raw_sn starts at zero.
            out = make_msg([dst_t, count * DT, data[2], data[3], data[4], data[5], data[6], data[7]])
            outbag.write(IMU_TOPIC, out, rospy.Time.from_sec(dst_t))
            count += 1
    return count


def write_high_mag(outbag, src_start, dst_start, duration):
    count = 0
    with rosbag.Bag(str(HIGH_IMU_BAG)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[HIGH_MAG_SOURCE_TOPIC]):
            data = list(msg.data)
            if len(data) <= 21:
                continue
            src_t = float(data[0])
            rel = src_t - src_start
            if rel < 0:
                continue
            if rel > duration:
                break
            dst_t = shifted_time(src_t, src_start, dst_start)
            # /L3/imu_origin already has +Z down acceleration, so keep magnetic axes unchanged.
            out = make_msg([dst_t, data[19], data[20], data[21]])
            outbag.write(MAG_TOPIC, out, rospy.Time.from_sec(dst_t))
            count += 1
    return count


def write_prepared_observations(outbag, dst_start, duration):
    counts = {GNSS_TOPIC: 0, RANGE_TOPIC: 0}
    dst_end = dst_start + duration
    with rosbag.Bag(str(PREPARED_BAG)) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[GNSS_TOPIC, RANGE_TOPIC]):
            t = stamp.to_sec()
            if t < dst_start:
                continue
            if t > dst_end:
                break
            data = list(msg.data)
            if data:
                data[0] = t
                msg = make_msg(data)
            outbag.write(topic, msg, stamp)
            counts[topic] += 1
    return counts


def main():
    if not PREPARED_BAG.exists():
        raise FileNotFoundError(PREPARED_BAG)
    if not HIGH_IMU_BAG.exists():
        raise FileNotFoundError(HIGH_IMU_BAG)

    OUT_BAG.parent.mkdir(parents=True, exist_ok=True)
    if OUT_BAG.exists():
        OUT_BAG.unlink()
    if OUT_META.exists():
        OUT_META.unlink()

    prepared_start = first_time(PREPARED_BAG, IMU_TOPIC)
    high_start = first_time(HIGH_IMU_BAG, IMU_TOPIC)
    high_end = last_time(HIGH_IMU_BAG, IMU_TOPIC)
    duration = high_end - high_start

    print(f"Prepared observations: {PREPARED_BAG}")
    print(f"High precision IMU  : {HIGH_IMU_BAG}")
    print(f"Output bag          : {OUT_BAG}")
    print(f"Mapped duration     : {duration:.6f} s")
    print(f"Output time span    : {prepared_start:.6f} -> {prepared_start + duration:.6f}")

    counts = {}
    with rosbag.Bag(str(OUT_BAG), "w", compression=rosbag.Compression.NONE) as outbag:
        counts[IMU_TOPIC] = write_high_imu(outbag, high_start, prepared_start, duration)
        counts[MAG_TOPIC] = write_high_mag(outbag, high_start, prepared_start, duration)
        counts.update(write_prepared_observations(outbag, prepared_start, duration))

    meta = {
        "prepared_bag": str(PREPARED_BAG),
        "high_precision_imu_bag": str(HIGH_IMU_BAG),
        "output_bag": str(OUT_BAG),
        "duration_s": duration,
        "prepared_start_time": prepared_start,
        "high_imu_start_time": high_start,
        "topic_counts": counts,
        "notes": [
            "This is a short bag because the high precision IMU source bag is only about 51 s long.",
            "/zh_origin is taken from the high precision /zh_origin topic and truncated to nav_kf columns 0..7.",
            "/mag_origin is generated from high precision /L3/imu_origin columns 19/20/21.",
            "/gnss_origin and /range are copied from the prepared bag over the mapped 51 s interval.",
            "The prepared bag contains processed UWB observations as /range, not raw /uwb.",
        ],
    }
    OUT_META.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(json.dumps(counts, indent=2, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
