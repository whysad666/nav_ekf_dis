#!/usr/bin/env python3
import json
import math
from pathlib import Path

import numpy as np
import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


ROOT = Path("/home/whysad/实验数据处理")
TARGET_BAG = ROOT / "prepared_bags/l3_mems_0611_2_nav_kf_input.bag"
RANGE_BAG = (
    ROOT
    / "prepared_bags/156-3_20260614_nav_kf_input_l3_aligned_200hz_trapz.bag"
)
OUT_BAG = ROOT / "prepared_bags/l3_mems_0611_2_nav_kf_input_with_156_3_range.bag"
OUT_META = OUT_BAG.with_suffix(".json")
TMP_BAG = OUT_BAG.with_suffix(".tmp.bag")

GNSS_TOPIC = "/gnss_origin"
RANGE_TOPIC = "/range"


def make_msg(values):
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return msg


def collapse_duplicate_x(x, y):
    order = np.argsort(x, kind="stable")
    x = np.asarray(x, dtype=float)[order]
    y = np.asarray(y, dtype=float)[order]
    unique_x, first, counts = np.unique(x, return_index=True, return_counts=True)
    if len(unique_x) == len(x):
        return x, y
    sums = np.add.reduceat(y, first)
    return unique_x, sums / counts


def load_gnss_clock(path):
    stamps = []
    utc_times = []
    data0_differences = []
    with rosbag.Bag(str(path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[GNSS_TOPIC]):
            data = list(msg.data)
            if len(data) <= 6:
                continue
            stamp_s = stamp.to_sec()
            utc = float(data[6])
            if not math.isfinite(stamp_s) or not math.isfinite(utc):
                continue
            stamps.append(stamp_s)
            utc_times.append(utc)
            if data and math.isfinite(float(data[0])):
                data0_differences.append(float(data[0]) - stamp_s)

    if len(stamps) < 2:
        raise RuntimeError(f"Not enough GNSS UTC samples in {path}")

    # UTC fields may be quantized. Average all ROS stamps that carry the same UTC.
    utc_to_stamp_x, utc_to_stamp_y = collapse_duplicate_x(utc_times, stamps)
    stamp_to_utc_x, stamp_to_utc_y = collapse_duplicate_x(stamps, utc_times)
    if len(utc_to_stamp_x) < 2 or len(stamp_to_utc_x) < 2:
        raise RuntimeError(f"GNSS UTC clock is degenerate in {path}")

    return {
        "stamp_to_utc_x": stamp_to_utc_x,
        "stamp_to_utc_y": stamp_to_utc_y,
        "utc_to_stamp_x": utc_to_stamp_x,
        "utc_to_stamp_y": utc_to_stamp_y,
        "sample_count": len(stamps),
        "utc_first": float(utc_to_stamp_x[0]),
        "utc_last": float(utc_to_stamp_x[-1]),
        "stamp_first": float(stamp_to_utc_x[0]),
        "stamp_last": float(stamp_to_utc_x[-1]),
        "data0_stamp_max_abs_s": (
            float(np.max(np.abs(data0_differences))) if data0_differences else None
        ),
    }


def compare_clocks(reference, target):
    reference_stamps = reference["stamp_to_utc_x"]
    reference_utc = reference["stamp_to_utc_y"]
    target_stamps = target["stamp_to_utc_x"]
    target_utc = target["stamp_to_utc_y"]
    if len(reference_stamps) != len(target_stamps):
        raise RuntimeError(
            "The range alignment timeline and target GNSS clocks have different lengths"
        )
    stamp_max_abs = float(np.max(np.abs(reference_stamps - target_stamps)))
    utc_max_abs = float(np.max(np.abs(reference_utc - target_utc)))
    if stamp_max_abs > 1e-9 or utc_max_abs > 1e-9:
        raise RuntimeError(
            "The range alignment timeline does not match the target L3 GNSS clock: "
            f"stamp max={stamp_max_abs}, UTC max={utc_max_abs}"
        )
    return {
        "sample_count": len(reference_stamps),
        "stamp_max_abs_difference_s": stamp_max_abs,
        "utc_max_abs_difference_s": utc_max_abs,
    }


def map_range_events(target_clock):
    events = []
    skipped = {"invalid": 0, "outside_target_clock": 0}
    stamp_corrections = []
    data0_input_differences = []

    with rosbag.Bag(str(RANGE_BAG)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[RANGE_TOPIC]):
            data = list(msg.data)
            source_stamp = stamp.to_sec()
            if len(data) < 6 or not all(math.isfinite(float(value)) for value in data):
                skipped["invalid"] += 1
                continue

            if data:
                data0_input_differences.append(float(data[0]) - source_stamp)

            if (
                source_stamp < target_clock["stamp_to_utc_x"][0]
                or source_stamp > target_clock["stamp_to_utc_x"][-1]
            ):
                skipped["outside_target_clock"] += 1
                continue

            # RANGE_BAG was already mapped through this exact L3 GNSS clock when
            # it was prepared. Preserve that timestamp to avoid a lossy second
            # interpolation through the 1 Hz 156-3 GNSS samples.
            target_stamp = source_stamp
            utc = float(
                np.interp(
                    target_stamp,
                    target_clock["stamp_to_utc_x"],
                    target_clock["stamp_to_utc_y"],
                )
            )

            data[0] = target_stamp
            events.append((target_stamp, make_msg(data), utc))
            stamp_corrections.append(target_stamp - source_stamp)

    events.sort(key=lambda item: item[0])
    return events, skipped, stamp_corrections, data0_input_differences


def write_merged_bag(events):
    counts = {}
    range_index = 0
    last_output_stamp = -math.inf

    with rosbag.Bag(str(TMP_BAG), "w", compression=rosbag.Compression.NONE) as outbag:
        with rosbag.Bag(str(TARGET_BAG)) as target:
            for topic, msg, stamp in target.read_messages():
                stamp_s = stamp.to_sec()
                while range_index < len(events) and events[range_index][0] <= stamp_s:
                    range_stamp, range_msg, _ = events[range_index]
                    if range_stamp < last_output_stamp:
                        raise RuntimeError("Merged output timestamps are not monotonic")
                    outbag.write(RANGE_TOPIC, range_msg, rospy.Time.from_sec(range_stamp))
                    counts[RANGE_TOPIC] = counts.get(RANGE_TOPIC, 0) + 1
                    last_output_stamp = range_stamp
                    range_index += 1

                if stamp_s < last_output_stamp:
                    raise RuntimeError("Target bag timestamps are not monotonic")
                outbag.write(topic, msg, stamp)
                counts[topic] = counts.get(topic, 0) + 1
                last_output_stamp = stamp_s

        while range_index < len(events):
            range_stamp, range_msg, _ = events[range_index]
            if range_stamp < last_output_stamp:
                raise RuntimeError("Trailing range timestamp is before the target bag end")
            outbag.write(RANGE_TOPIC, range_msg, rospy.Time.from_sec(range_stamp))
            counts[RANGE_TOPIC] = counts.get(RANGE_TOPIC, 0) + 1
            last_output_stamp = range_stamp
            range_index += 1

    TMP_BAG.replace(OUT_BAG)
    return counts


def describe(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return {
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "max": float(np.max(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
    }


def load_topic_stamps(path, topic):
    stamps = []
    data0_differences = []
    with rosbag.Bag(str(path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            stamp_s = stamp.to_sec()
            stamps.append(stamp_s)
            data = list(getattr(msg, "data", []))
            if data and math.isfinite(float(data[0])):
                data0_differences.append(float(data[0]) - stamp_s)
    if not stamps:
        raise RuntimeError(f"No messages on {topic} in {path}")
    return np.asarray(stamps, dtype=float), data0_differences


def nearest_absolute_differences(query, reference):
    query = np.asarray(query, dtype=float)
    reference = np.asarray(reference, dtype=float)
    indices = np.searchsorted(reference, query)
    left = reference[np.clip(indices - 1, 0, len(reference) - 1)]
    right = reference[np.clip(indices, 0, len(reference) - 1)]
    return np.minimum(np.abs(query - left), np.abs(right - query))


def describe_absolute(values):
    values = np.abs(np.asarray(values, dtype=float))
    if not len(values):
        return None
    return {
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def main():
    for path in (TARGET_BAG, RANGE_BAG):
        if not path.exists():
            raise FileNotFoundError(path)
    print(f"Target L3 MEMS bag: {TARGET_BAG}")
    print(f"Range source bag  : {RANGE_BAG}")
    print(f"Output bag        : {OUT_BAG}")

    range_meta_path = RANGE_BAG.with_suffix(".json")
    if not range_meta_path.exists():
        raise FileNotFoundError(range_meta_path)
    range_meta = json.loads(range_meta_path.read_text(encoding="utf-8"))
    if "l3_timeline_bag" not in range_meta:
        raise RuntimeError("Range metadata does not identify its L3 UTC timeline bag")
    range_timeline_bag = Path(range_meta["l3_timeline_bag"])
    if not range_timeline_bag.exists():
        raise FileNotFoundError(range_timeline_bag)

    range_timeline_clock = load_gnss_clock(range_timeline_bag)
    target_clock = load_gnss_clock(TARGET_BAG)
    clock_identity = compare_clocks(range_timeline_clock, target_clock)

    events, skipped, corrections, input_data0_differences = map_range_events(target_clock)
    if not events:
        raise RuntimeError("No /range messages remain after UTC alignment")

    counts = write_merged_bag(events)
    imu_stamps, imu_data0_differences = load_topic_stamps(TARGET_BAG, "/zh_origin")
    range_stamps = np.asarray([event[0] for event in events], dtype=float)
    gnss_stamps = target_clock["stamp_to_utc_x"]
    range_to_imu = nearest_absolute_differences(range_stamps, imu_stamps)
    range_to_gnss = nearest_absolute_differences(range_stamps, gnss_stamps)

    meta = {
        "target_bag": str(TARGET_BAG),
        "range_source_bag": str(RANGE_BAG),
        "output_bag": str(OUT_BAG),
        "alignment": {
            "method": "preserve source /range stamps after validating the original UTC-to-L3 timeline against the target GNSS clock",
            "range_preparation_metadata": str(range_meta_path),
            "range_l3_timeline_bag": str(range_timeline_bag),
            "timeline_clock_identity": clock_identity,
            "target_clock": {
                key: value
                for key, value in target_clock.items()
                if not isinstance(value, np.ndarray)
            },
            "target_minus_source_stamp_s": describe(corrections),
            "source_range_data0_minus_bag_stamp_s": describe(input_data0_differences),
        },
        "range": {
            "input_count": int(sum(1 for _ in events) + sum(skipped.values())),
            "output_count": len(events),
            "first_target_stamp": events[0][0],
            "last_target_stamp": events[-1][0],
            "first_utc": events[0][2],
            "last_utc": events[-1][2],
            "skipped": skipped,
            "data0_rewritten_to_target_stamp": True,
        },
        "topic_counts": counts,
        "synchronization": {
            "clock_definition": "bag stamp and data[0] use the common L3 absolute time axis",
            "range_data0_minus_bag_stamp_abs_s": describe_absolute(
                input_data0_differences
            ),
            "imu_data0_minus_bag_stamp_abs_s": describe_absolute(
                imu_data0_differences
            ),
            "gnss_data0_minus_bag_stamp_max_abs_s": target_clock[
                "data0_stamp_max_abs_s"
            ],
            "range_to_nearest_imu_abs_s": describe_absolute(range_to_imu),
            "range_to_nearest_gnss_abs_s": describe_absolute(range_to_gnss),
            "all_range_inside_imu_time_span": bool(
                range_stamps[0] >= imu_stamps[0]
                and range_stamps[-1] <= imu_stamps[-1]
            ),
            "note": "Nearest-sample differences reflect asynchronous sampling; range timestamps are not snapped to IMU or GNSS samples.",
        },
        "notes": [
            "The original L3 MEMS bag is not modified.",
            "Only /range is imported from the 156-3 prepared bag.",
            "All original /zh_origin and /gnss_origin messages are preserved.",
            "The source prepared bag was already mapped through GNSS data[6] UTC to the L3 timeline.",
            "The original L3 timeline GNSS clock is byte-for-value identical in stamp and UTC arrays to the target MEMS GNSS clock.",
            "Range timestamps are therefore copied exactly; a second 1 Hz UTC interpolation would add avoidable timing error.",
            "Range anchor positions and ID mapping remain exactly as stored in the source /range messages.",
        ],
    }
    OUT_META.write_text(
        json.dumps(meta, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(json.dumps(meta["range"], ensure_ascii=False, indent=2, sort_keys=True))
    print(json.dumps(meta["alignment"]["target_minus_source_stamp_s"], indent=2))
    print(json.dumps(meta["synchronization"], ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
