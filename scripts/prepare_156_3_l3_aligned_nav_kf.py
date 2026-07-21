#!/usr/bin/env python3
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


ROOT = Path("/media/whysad/Lenovo1/260614铜官窑实验/extracted_9bags")
MAIN_BAG = ROOT / "156-3_uwb_imu_gnss_20260614.bag"
PEER_BAGS = {
    0: ROOT / "154-3_uwb_imu_gnss_20260614.bag",
    1: ROOT / "153-3_uwb_imu_gnss_20260614.bag",
}
L3_BAG = Path("/home/whysad/实验数据处理/prepared_bags/l3_0611_2_nav_kf_input.bag")
OUT_BAG = Path(
    "/home/whysad/实验数据处理/prepared_bags/"
    "156-3_20260614_nav_kf_input_l3_aligned_200hz_trapz.bag"
)
OUT_META = OUT_BAG.with_suffix(".json")

TOPIC_IMU_RAW = "/imu_origin"
TOPIC_GNSS = "/gnss_origin"
TOPIC_UWB = "/uwb"
TOPIC_IMU = "/zh_origin"
TOPIC_MAG = "/mag_origin"
TOPIC_RANGE = "/range"

RATE_HZ = 200.0
DT = 1.0 / RATE_HZ
MAIN_UWB_ID = 2
PEER_TO_EKF_ID = {0: 4, 1: 1, 3: 3}

# Node 155 has no GNSS bag. This coordinate was read from 155/3位置.jpg.
STATIC_ANCHORS_DEG = {
    3: {
        "node": "155",
        "lat_deg": 28.453392189666666,
        "lon_deg": 112.81127966866667,
        "alt_m": 34.7828,
        "source": "155/3位置.jpg GGA fix",
    }
}


class Progress:
    def __init__(self, label, total):
        self.label = label
        self.total = max(int(total), 1)
        self.value = 0
        self.step = max(1, self.total // 100)
        self.last = -1

    def update(self, delta=1):
        self.value += delta
        if self.value == self.total or self.value - self.last >= self.step:
            self.last = self.value
            ratio = min(1.0, self.value / self.total)
            print(
                f"\r{self.label}: {self.value}/{self.total} ({ratio * 100:5.1f}%)",
                end="",
                flush=True,
            )

    def close(self):
        if self.value < self.total:
            self.update(self.total - self.value)
        print()


def make_msg(values):
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return msg


def interp_extrap(x, xp, fp):
    x_arr = np.asarray(x, dtype=float)
    xp = np.asarray(xp, dtype=float)
    fp = np.asarray(fp, dtype=float)
    if len(xp) < 2:
        raise RuntimeError("at least two clock samples are required")
    if np.any(np.diff(xp) <= 0):
        raise RuntimeError("clock samples are not strictly increasing")

    y = np.interp(x_arr, xp, fp)
    left_slope = (fp[1] - fp[0]) / (xp[1] - xp[0])
    right_slope = (fp[-1] - fp[-2]) / (xp[-1] - xp[-2])
    y = np.where(x_arr < xp[0], fp[0] + (x_arr - xp[0]) * left_slope, y)
    y = np.where(x_arr > xp[-1], fp[-1] + (x_arr - xp[-1]) * right_slope, y)
    return float(y) if x_arr.ndim == 0 else y


def load_gnss_clock(path):
    stamps = []
    utc_times = []
    with rosbag.Bag(str(path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[TOPIC_GNSS]):
            data = list(msg.data)
            if len(data) <= 6:
                continue
            utc = float(data[6])
            if math.isfinite(utc):
                stamps.append(stamp.to_sec())
                utc_times.append(utc)
    if len(stamps) < 2:
        raise RuntimeError(f"not enough GNSS clock samples in {path}")
    return np.asarray(stamps), np.asarray(utc_times)


def load_l3_bounds_and_clock():
    imu_first = None
    imu_last = None
    with rosbag.Bag(str(L3_BAG)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[TOPIC_IMU]):
            t = float(msg.data[0]) if len(msg.data) else stamp.to_sec()
            if imu_first is None:
                imu_first = t
            imu_last = t
    if imu_first is None or imu_last is None:
        raise RuntimeError(f"no {TOPIC_IMU} in {L3_BAG}")
    l3_stamps, l3_utc = load_gnss_clock(L3_BAG)
    return float(imu_first), float(imu_last), l3_stamps, l3_utc


def source_stamp_to_utc(stamps, source_stamps, source_utc):
    return interp_extrap(stamps, source_stamps, source_utc)


def utc_to_l3_stamp(utc_times, l3_utc, l3_stamps):
    return interp_extrap(utc_times, l3_utc, l3_stamps)


def load_main_imu(source_stamps, source_utc, l3_utc, l3_stamps):
    raw_stamps = []
    rates = []
    mags = []
    frames = []

    with rosbag.Bag(str(MAIN_BAG)) as bag:
        total = bag.get_message_count(topic_filters=TOPIC_IMU_RAW)
        progress = Progress("read 156-3 IMU/MAG", total)
        for _, msg, stamp in bag.read_messages(topics=[TOPIC_IMU_RAW]):
            data = list(msg.data)
            if len(data) <= 21:
                progress.update()
                continue
            values = [data[i] for i in (4, 5, 6, 1, 2, 3, 19, 20, 21)]
            if not all(math.isfinite(float(v)) for v in values):
                progress.update()
                continue

            gx, gy, gz, ax, ay, az, mx, my, mz = map(float, values)
            raw_stamps.append(stamp.to_sec())
            # Aligned flight dynamics show that the 156-3 sensor is mounted with
            # x/y exchanged relative to the L3/nav_kf body axes.
            rates.append((-gy, gx, -gz, -ay, ax, -az))
            mags.append((-my, mx, -mz))
            frames.append(int(round(data[14])) & 0xFF if len(data) > 14 else -1)
            progress.update()
        progress.close()

    raw_stamps = np.asarray(raw_stamps, dtype=float)
    rates = np.asarray(rates, dtype=float)
    mags = np.asarray(mags, dtype=float)
    frames = np.asarray(frames, dtype=int)
    utc_times = source_stamp_to_utc(raw_stamps, source_stamps, source_utc)
    target_times = utc_to_l3_stamp(utc_times, l3_utc, l3_stamps)

    keep = np.r_[True, np.diff(target_times) > 1e-9]
    dropped_non_increasing = int(np.sum(~keep))
    return (
        target_times[keep],
        rates[keep],
        mags[keep],
        frames[keep],
        raw_stamps[keep],
        dropped_non_increasing,
    )


def interp_columns(new_times, old_times, old_values):
    return np.column_stack(
        [np.interp(new_times, old_times, old_values[:, col]) for col in range(old_values.shape[1])]
    )


def build_resampled_imu(target_times, rates, mags, l3_first, l3_last):
    overlap_end = min(float(target_times[-1]), float(l3_last))
    first_index = int(math.ceil((target_times[0] - l3_first) / DT - 1e-9))
    last_edge_index = int(math.floor((overlap_end - l3_first) / DT + 1e-9))
    if last_edge_index <= first_index:
        raise RuntimeError("156-3 and L3 IMU timelines do not overlap")

    edge_indices = np.arange(first_index, last_edge_index + 1, dtype=np.int64)
    edge_times = l3_first + edge_indices.astype(float) * DT
    sample_times = edge_times[:-1]
    edge_rates = interp_columns(edge_times, target_times, rates)
    increments = 0.5 * (edge_rates[:-1] + edge_rates[1:]) * DT
    mag_samples = interp_columns(sample_times, target_times, mags)
    return sample_times, increments, mag_samples


def load_peer_track(path):
    utc_times = []
    positions = []
    with rosbag.Bag(str(path)) as bag:
        for _, msg, _ in bag.read_messages(topics=[TOPIC_GNSS]):
            data = list(msg.data)
            if len(data) <= 6:
                continue
            values = [float(data[i]) for i in (6, 3, 2, 4)]
            if not all(math.isfinite(value) for value in values):
                continue
            utc, lat_deg, lon_deg, alt = values
            utc_times.append(utc)
            positions.append((math.radians(lat_deg), math.radians(lon_deg), alt))
    if len(utc_times) < 2:
        raise RuntimeError(f"not enough peer GNSS fixes in {path}")
    return {"utc": np.asarray(utc_times), "position": np.asarray(positions)}


def interp_position(track, utc):
    times = track["utc"]
    if utc < times[0] or utc > times[-1]:
        return None
    position = track["position"]
    return tuple(float(np.interp(utc, times, position[:, col])) for col in range(3))


def static_anchor_position(peer_id):
    anchor = STATIC_ANCHORS_DEG[peer_id]
    return (
        math.radians(anchor["lat_deg"]),
        math.radians(anchor["lon_deg"]),
        float(anchor["alt_m"]),
    )


def parse_uwb(data):
    if len(data) < 11:
        return []
    declared_count = int(round(data[5]))
    count = min(declared_count, max(0, (len(data) - 6) // 5))
    observations = []
    for index in range(count):
        base = 6 + index * 5
        observations.append(
            {
                "peer_id": int(round(data[base])),
                "range_m": float(data[base + 2]),
            }
        )
    return observations


def build_gnss_events(source_stamps, source_utc, l3_utc, l3_stamps, start, end):
    events = []
    with rosbag.Bag(str(MAIN_BAG)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[TOPIC_GNSS]):
            data = list(msg.data)
            if len(data) <= 6:
                continue
            utc = float(data[6])
            target_time = utc_to_l3_stamp(utc, l3_utc, l3_stamps)
            if target_time < start or target_time > end:
                continue
            data[0] = target_time
            events.append((target_time, TOPIC_GNSS, make_msg(data)))
    return events


def build_range_events(source_stamps, source_utc, l3_utc, l3_stamps, tracks, start, end):
    events = []
    raw_peer_counts = Counter()
    used_peer_counts = Counter()
    skipped = Counter()

    with rosbag.Bag(str(MAIN_BAG)) as bag:
        total = bag.get_message_count(topic_filters=TOPIC_UWB)
        progress = Progress("convert 156-3 UWB", total)
        for _, msg, stamp in bag.read_messages(topics=[TOPIC_UWB]):
            data = list(msg.data)
            if len(data) < 11 or int(round(data[3])) != MAIN_UWB_ID:
                skipped["invalid_message"] += 1
                progress.update()
                continue

            utc = source_stamp_to_utc(stamp.to_sec(), source_stamps, source_utc)
            target_time = utc_to_l3_stamp(utc, l3_utc, l3_stamps)
            if target_time < start or target_time > end:
                skipped["outside_l3_overlap"] += 1
                progress.update()
                continue

            out = [target_time]
            for obs in parse_uwb(data):
                peer_id = obs["peer_id"]
                range_m = obs["range_m"]
                raw_peer_counts[str(peer_id)] += 1
                if peer_id not in PEER_TO_EKF_ID:
                    skipped["unknown_peer"] += 1
                    continue
                if not math.isfinite(range_m) or range_m < 1.0:
                    skipped["invalid_range"] += 1
                    continue

                if peer_id in STATIC_ANCHORS_DEG:
                    position = static_anchor_position(peer_id)
                else:
                    position = interp_position(tracks[peer_id], utc)
                if position is None:
                    skipped["peer_position_out_of_range"] += 1
                    continue

                out.extend(
                    [
                        float(PEER_TO_EKF_ID[peer_id]),
                        position[0],
                        position[1],
                        position[2],
                        range_m,
                    ]
                )
                used_peer_counts[str(peer_id)] += 1

            if len(out) == 1:
                skipped["empty_output"] += 1
            else:
                events.append((target_time, TOPIC_RANGE, make_msg(out)))
            progress.update()
        progress.close()

    return events, dict(raw_peer_counts), dict(used_peer_counts), dict(skipped)


def write_output(sample_times, increments, mag_samples, gnss_events, range_events):
    observation_events = sorted(gnss_events + range_events, key=lambda item: item[0])
    obs_index = 0
    counts = Counter()
    progress = Progress("write prepared bag", len(sample_times))

    with rosbag.Bag(str(OUT_BAG), "w", compression=rosbag.Compression.NONE) as outbag:
        for index, target_time in enumerate(sample_times):
            while obs_index < len(observation_events) and observation_events[obs_index][0] <= target_time:
                event_time, topic, msg = observation_events[obs_index]
                outbag.write(topic, msg, rospy.Time.from_sec(event_time))
                counts[topic] += 1
                obs_index += 1

            imu_msg = make_msg(
                [
                    target_time,
                    index * DT,
                    increments[index, 0],
                    increments[index, 1],
                    increments[index, 2],
                    increments[index, 3],
                    increments[index, 4],
                    increments[index, 5],
                ]
            )
            mag_msg = make_msg(
                [target_time, mag_samples[index, 0], mag_samples[index, 1], mag_samples[index, 2]]
            )
            stamp = rospy.Time.from_sec(float(target_time))
            outbag.write(TOPIC_IMU, imu_msg, stamp)
            outbag.write(TOPIC_MAG, mag_msg, stamp)
            counts[TOPIC_IMU] += 1
            counts[TOPIC_MAG] += 1
            progress.update()

        while obs_index < len(observation_events):
            event_time, topic, msg = observation_events[obs_index]
            outbag.write(topic, msg, rospy.Time.from_sec(event_time))
            counts[topic] += 1
            obs_index += 1
        progress.close()
    return dict(counts)


def main():
    for path in [MAIN_BAG, L3_BAG, *PEER_BAGS.values()]:
        if not path.exists():
            raise FileNotFoundError(path)
    OUT_BAG.parent.mkdir(parents=True, exist_ok=True)
    if OUT_BAG.exists():
        OUT_BAG.unlink()
    if OUT_META.exists():
        OUT_META.unlink()

    print(f"Main input : {MAIN_BAG}")
    print(f"L3 timeline: {L3_BAG}")
    print(f"Output     : {OUT_BAG}")

    source_stamps, source_utc = load_gnss_clock(MAIN_BAG)
    l3_first, l3_last, l3_stamps, l3_utc = load_l3_bounds_and_clock()
    target_times, rates, mags, frames, raw_stamps, dropped = load_main_imu(
        source_stamps, source_utc, l3_utc, l3_stamps
    )
    sample_times, increments, mag_samples = build_resampled_imu(
        target_times, rates, mags, l3_first, l3_last
    )

    start = float(sample_times[0])
    end = float(sample_times[-1])
    tracks = {peer_id: load_peer_track(path) for peer_id, path in PEER_BAGS.items()}
    gnss_events = build_gnss_events(source_stamps, source_utc, l3_utc, l3_stamps, start, end)
    range_events, raw_peers, used_peers, range_skipped = build_range_events(
        source_stamps, source_utc, l3_utc, l3_stamps, tracks, start, end
    )
    counts = write_output(sample_times, increments, mag_samples, gnss_events, range_events)

    source_gaps = np.diff(raw_stamps)
    frame_delta = (frames[1:] - frames[:-1]) % 256
    meta = {
        "input_bag": str(MAIN_BAG),
        "l3_timeline_bag": str(L3_BAG),
        "output_bag": str(OUT_BAG),
        "rate_hz": RATE_HZ,
        "axis_conversion": "156-3 source to L3/nav_kf: x=-source_y, y=source_x, z=-source_z",
        "topic_counts": counts,
        "output_time": {
            "first": start,
            "last": end,
            "duration_s": end - start,
            "l3_first": l3_first,
            "l3_last": l3_last,
            "start_after_l3_s": start - l3_first,
        },
        "utc": {
            "main_first": float(source_utc[0]),
            "main_last": float(source_utc[-1]),
            "l3_first": float(l3_utc[0]),
            "l3_last": float(l3_utc[-1]),
            "main_start_after_l3_s": float(source_utc[0] - l3_utc[0]),
        },
        "imu_source": {
            "valid_messages": int(len(raw_stamps)),
            "resampled_messages": int(len(sample_times)),
            "dropped_non_increasing_timestamps": dropped,
            "median_dt_s": float(np.median(source_gaps)),
            "max_dt_s": float(np.max(source_gaps)),
            "gaps_over_20ms": int(np.sum(source_gaps > 0.02)),
            "frame_delta_not_one": int(np.sum(frame_delta != 1)),
            "estimated_lost_frames": int(np.sum(np.maximum(frame_delta - 1, 0))),
        },
        "mag_source": {
            "fields": [19, 20, 21],
            "units": "source units documented as microtesla",
            "raw_median_norm": float(np.median(np.linalg.norm(mags, axis=1))),
            "warning": "raw magnetometer is not hard/soft-iron calibrated in this preparation",
        },
        "range": {
            "main_raw_uwb_id": MAIN_UWB_ID,
            "peer_to_ekf_id": {str(k): v for k, v in PEER_TO_EKF_ID.items()},
            "peer_bags": {str(k): str(v) for k, v in PEER_BAGS.items()},
            "static_anchors_deg": STATIC_ANCHORS_DEG,
            "raw_peer_counts": raw_peers,
            "used_peer_counts": used_peers,
            "skipped": range_skipped,
        },
        "notes": [
            "All output timestamps are mapped to the L3 prepared-bag clock through GNSS UTC data[6].",
            "The output is trimmed to the time interval shared by 156-3 and L3.",
            "IMU rate samples are linearly interpolated at 200Hz edges and converted to dtheta/dvel by trapezoidal integration.",
            "Long source IMU gaps are interpolated; this preserves continuity but cannot recover real motion inside a missing interval.",
            "No frame_index is appended because the output contains per-interval increments on a fixed 200Hz grid.",
            "GNSS data[6] is preserved as the experiment UTC time; GNSS data[0] is rewritten to the L3 clock.",
            "MAG is placed on the same 200Hz time grid as IMU and converted to FRD.",
            "The inertial-axis mapping was verified against aligned L3 flight dynamics; source x/y are not the same as L3/nav_kf x/y.",
            "Range peer positions for IDs 0 and 1 are interpolated from 154-3 and 153-3 GNSS; ID 3 is the fixed 155 coordinate.",
        ],
    }
    OUT_META.write_text(json.dumps(meta, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(json.dumps(meta, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
