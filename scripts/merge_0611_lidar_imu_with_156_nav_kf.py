#!/usr/bin/env python3
import bisect
import json
import math
import os
from pathlib import Path

import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


HIGH_IMU_BAG = Path("/media/whysad/Lenovo1/260614铜官窑实验/l型/0611_2.bag")
TARGET_BAG = Path("/media/whysad/Lenovo1/260611白水实验/1.156/0611-2.bag")
ANCHOR_ROOT = Path("/media/whysad/Lenovo1/260611白水实验")
OUT_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0611-2_lidar_imu_nav_kf_input.bag")
OUT_META = OUT_BAG.with_suffix(".json")

TOPIC_IMU = "/zh_origin"
TOPIC_HIGH_GNSS = "/L3/gnss_origin"
TOPIC_GNSS_ORIGIN = "/gnss_origin"
TOPIC_MAG = "/mag_origin"
TOPIC_RANGE = "/range"
TOPIC_TARGET_IMU_ORIGIN = "/imu_origin"
TOPIC_TARGET_UWB = "/uwb"

IMU_RATE_HZ = 200.0
DT = 1.0 / IMU_RATE_HZ
MAIN_UWB_ID = 2

# Keep the same raw-UWB to EKF-id mapping used for the 0614 prepared bag.
PEER_TO_EKF_ID = {
    0: 4,
    1: 1,
    3: 3,
}

PEER_TO_NODE = {
    0: "1.154",
    1: "1.153",
    3: "1.155",
}

R_EARTH_A = 6378137.0
R_EARTH_E2 = 6.69437999014e-3


class Progress:
    def __init__(self, label, total=0, step=5000):
        self.label = label
        self.total = int(total or 0)
        self.step = max(1, int(step))
        self.value = 0
        self.last = 0

    def tick(self, n=1):
        self.value += n
        if self.value - self.last >= self.step:
            self.last = self.value
            self.render()

    def render(self):
        if self.total > 0:
            pct = min(100.0, self.value * 100.0 / self.total)
            print(f"\r{self.label}: {self.value}/{self.total} ({pct:5.1f}%)", end="", flush=True)
        else:
            print(f"\r{self.label}: {self.value}", end="", flush=True)

    def close(self):
        self.render()
        print()


def make_msg(values):
    msg = Float64MultiArray()
    msg.data = [float(v) for v in values]
    return msg


def first_array_message(path, topic):
    print(f"read first {topic}: {path}", flush=True)
    with rosbag.Bag(str(path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            data = list(msg.data)
            if data:
                return data, stamp.to_sec()
    raise RuntimeError(f"No non-empty message found on {topic} in {path}")


def blh_distance_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * R_EARTH_A * math.asin(min(1.0, math.sqrt(h)))


def compute_time_map():
    high_gnss, high_stamp = first_array_message(HIGH_IMU_BAG, TOPIC_HIGH_GNSS)
    target_gnss, target_stamp = first_array_message(TARGET_BAG, TOPIC_GNSS_ORIGIN)

    if len(high_gnss) <= 6 or len(target_gnss) <= 6:
        raise RuntimeError("GNSS origin message does not contain UTC field data[6]")

    high_utc = float(high_gnss[6])
    target_utc = float(target_gnss[6])

    env_offset = os.environ.get("HIGH_TO_TARGET_UTC_OFFSET")
    if env_offset is None:
        # Default manual rule: align the first GNSS UTC epoch of both bags.
        utc_offset = target_utc - high_utc
        offset_source = "first_gnss_utc"
    else:
        utc_offset = float(env_offset)
        offset_source = "HIGH_TO_TARGET_UTC_OFFSET"

    high_ros_to_utc = high_utc - float(high_gnss[0])
    target_utc_to_ros = float(target_gnss[0]) - target_utc
    high_ros_to_target_ros = high_ros_to_utc + utc_offset + target_utc_to_ros

    high_llh = (float(high_gnss[3]), float(high_gnss[2]), float(high_gnss[4]))
    target_llh = (float(target_gnss[3]), float(target_gnss[2]), float(target_gnss[4]))
    first_gnss_distance = blh_distance_m(high_llh, target_llh)

    print(f"high first GNSS   time={high_gnss[0]:.6f} utc={high_utc:.3f} llh={high_llh}")
    print(f"target first GNSS time={target_gnss[0]:.6f} utc={target_utc:.3f} llh={target_llh}")
    print(f"UTC offset high->target: {utc_offset:.6f} s ({offset_source})")
    print(f"ROS time offset high->target: {high_ros_to_target_ros:.6f} s")
    if first_gnss_distance > 1000.0:
        print(f"WARNING: first GNSS positions differ by {first_gnss_distance:.1f} m")

    return {
        "high_first_gnss_time": float(high_gnss[0]),
        "target_first_gnss_time": float(target_gnss[0]),
        "high_first_utc": high_utc,
        "target_first_utc": target_utc,
        "utc_offset_high_to_target": utc_offset,
        "utc_offset_source": offset_source,
        "high_ros_to_target_ros": high_ros_to_target_ros,
        "first_gnss_distance_m": first_gnss_distance,
        "high_first_llh": high_llh,
        "target_first_llh": target_llh,
    }


def target_window(time_map):
    with rosbag.Bag(str(TARGET_BAG)) as target, rosbag.Bag(str(HIGH_IMU_BAG)) as high:
        target_start = target.get_start_time()
        target_end = target.get_end_time()
        high_start_mapped = high.get_start_time() + time_map["high_ros_to_target_ros"]
        high_end_mapped = high.get_end_time() + time_map["high_ros_to_target_ros"]

    start = max(target_start, high_start_mapped)
    end = min(target_end, high_end_mapped)
    if end <= start:
        raise RuntimeError(f"No overlap after time mapping: start={start}, end={end}")
    print(f"output window: {start:.6f} -> {end:.6f} ({end - start:.3f} s)")
    return start, end


def first_uwb_reference(path):
    with rosbag.Bag(str(path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[TOPIC_TARGET_UWB]):
            data = list(msg.data)
            if len(data) >= 6:
                return {
                    "bag_time": stamp.to_sec(),
                    "msg_time": float(data[0]),
                    "uwb_time": float(data[1]) / 1000.0,
                    "self_id": int(round(data[3])),
                }
    raise RuntimeError(f"No /uwb message in {path}")


def common_time_from_bag_time(ref, bag_time):
    return ref["uwb_time"] + (bag_time - ref["bag_time"])


def load_anchor_track(peer_id):
    node = PEER_TO_NODE[peer_id]
    path = ANCHOR_ROOT / node / "0611-2.bag"
    ref = first_uwb_reference(path)
    times = []
    positions = []
    print(f"load anchor peer {peer_id} from {path}", flush=True)
    with rosbag.Bag(str(path)) as bag:
        total = bag.get_message_count(topic_filters="/gnss")
        progress = Progress(f"load {node}/gnss", total, step=max(1, total // 40 if total else 100))
        for _, msg, stamp in bag.read_messages(topics=["/gnss"]):
            if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
                progress.tick()
                continue
            times.append(common_time_from_bag_time(ref, stamp.to_sec()))
            positions.append((math.radians(msg.latitude), math.radians(msg.longitude), float(msg.altitude)))
            progress.tick()
        progress.close()
    if len(times) < 2:
        raise RuntimeError(f"Not enough GNSS samples for anchor peer {peer_id}: {len(times)}")
    return {"node": node, "ref": ref, "times": times, "positions": positions}


def interp_position(track, t_common):
    times = track["times"]
    positions = track["positions"]
    if t_common < times[0] or t_common > times[-1]:
        return None
    idx = bisect.bisect_left(times, t_common)
    if idx <= 0:
        return positions[0]
    if idx >= len(times):
        return positions[-1]
    t0, t1 = times[idx - 1], times[idx]
    p0, p1 = positions[idx - 1], positions[idx]
    if t1 <= t0:
        return p0
    a = (t_common - t0) / (t1 - t0)
    return tuple(p0[j] + a * (p1[j] - p0[j]) for j in range(3))


def parse_uwb_ranges(data):
    if len(data) < 11:
        return []
    declared_count = int(round(data[5]))
    groups = []
    max_by_len = max(0, (len(data) - 6) // 5)
    for k in range(min(declared_count, max_by_len)):
        base = 6 + 5 * k
        groups.append(
            {
                "peer_id": int(round(data[base])),
                "peer_role": int(round(data[base + 1])),
                "range_m": float(data[base + 2]),
                "extra0": float(data[base + 3]),
                "extra1": float(data[base + 4]),
            }
        )
    return groups


def build_anchor_tracks():
    tracks = {}
    for peer_id in sorted(PEER_TO_NODE):
        tracks[peer_id] = load_anchor_track(peer_id)
    return tracks


def next_or_none(iterator):
    try:
        return next(iterator)
    except StopIteration:
        return None


def high_imu_iterator(time_offset, start, end, stats):
    count = 0
    written = 0
    with rosbag.Bag(str(HIGH_IMU_BAG)) as bag:
        total = bag.get_message_count(topic_filters=TOPIC_IMU)
        progress = Progress("read high /zh_origin", total, step=5000)
        for _, msg, stamp in bag.read_messages(topics=[TOPIC_IMU]):
            progress.tick()
            data = list(msg.data)
            if len(data) < 8:
                continue
            src_t = float(data[0])
            out_t = src_t + time_offset
            if out_t < start:
                count += 1
                continue
            if out_t > end:
                break
            out = make_msg([out_t, written * DT, data[2], data[3], data[4], data[5], data[6], data[7]])
            written += 1
            yield TOPIC_IMU, out, rospy.Time.from_sec(out_t)
            count += 1
        progress.close()
    stats["imu_messages"] = written


def target_observation_iterator(start, end, tracks, stats):
    counts = {
        "gnss_origin_messages": 0,
        "mag_origin_messages": 0,
        "range_messages": 0,
        "range_anchor_observations": 0,
    }
    skipped = {
        "uwb_empty": 0,
        "uwb_not_main": 0,
        "unknown_peer": 0,
        "invalid_range": 0,
        "out_of_track": 0,
        "empty_range_output": 0,
        "outside_window": 0,
    }
    peer_counts = {}

    with rosbag.Bag(str(TARGET_BAG)) as bag:
        total = (
            bag.get_message_count(topic_filters=TOPIC_GNSS_ORIGIN)
            + bag.get_message_count(topic_filters=TOPIC_TARGET_IMU_ORIGIN)
            + bag.get_message_count(topic_filters=TOPIC_TARGET_UWB)
        )
        progress = Progress("read target obs", total, step=5000)
        for topic, msg, stamp in bag.read_messages(topics=[TOPIC_GNSS_ORIGIN, TOPIC_TARGET_IMU_ORIGIN, TOPIC_TARGET_UWB]):
            progress.tick()

            if topic == TOPIC_GNSS_ORIGIN:
                data = list(msg.data)
                if not data:
                    continue
                t = float(data[0])
                if t < start or t > end:
                    skipped["outside_window"] += 1
                    continue
                counts["gnss_origin_messages"] += 1
                yield TOPIC_GNSS_ORIGIN, make_msg(data), rospy.Time.from_sec(t)
                continue

            if topic == TOPIC_TARGET_IMU_ORIGIN:
                data = list(msg.data)
                if len(data) <= 21:
                    continue
                mag_x, mag_y, mag_z = float(data[19]), float(data[20]), float(data[21])
                if not all(math.isfinite(v) for v in (mag_x, mag_y, mag_z)):
                    continue
                t = stamp.to_sec()
                if t < start or t > end:
                    skipped["outside_window"] += 1
                    continue
                # 156 raw IMU/mag is in the same body frame as the original ROS IMU.
                # Prepared /zh_origin is FRD, so flip Y/Z for magnetic axes.
                counts["mag_origin_messages"] += 1
                yield TOPIC_MAG, make_msg([t, mag_x, -mag_y, -mag_z]), rospy.Time.from_sec(t)
                continue

            data = list(msg.data)
            if len(data) < 6:
                skipped["uwb_empty"] += 1
                continue
            t_out = float(data[0]) if math.isfinite(float(data[0])) else stamp.to_sec()
            if t_out < start or t_out > end:
                skipped["outside_window"] += 1
                continue
            if int(round(data[3])) != MAIN_UWB_ID:
                skipped["uwb_not_main"] += 1
                continue

            obs_list = parse_uwb_ranges(data)
            if not obs_list:
                skipped["uwb_empty"] += 1
                continue

            t_common = float(data[1]) / 1000.0
            out_data = [t_out]
            for obs in obs_list:
                peer_id = obs["peer_id"]
                range_m = obs["range_m"]
                if peer_id not in PEER_TO_EKF_ID or peer_id not in tracks:
                    skipped["unknown_peer"] += 1
                    continue
                if not math.isfinite(range_m) or range_m < 1.0:
                    skipped["invalid_range"] += 1
                    continue
                pos = interp_position(tracks[peer_id], t_common)
                if pos is None:
                    skipped["out_of_track"] += 1
                    continue
                out_data.extend([float(PEER_TO_EKF_ID[peer_id]), pos[0], pos[1], pos[2], range_m])
                peer_counts[str(peer_id)] = peer_counts.get(str(peer_id), 0) + 1

            if len(out_data) == 1:
                skipped["empty_range_output"] += 1
                continue

            counts["range_messages"] += 1
            counts["range_anchor_observations"] += (len(out_data) - 1) // 5
            yield TOPIC_RANGE, make_msg(out_data), rospy.Time.from_sec(t_out)

        progress.close()

    stats.update(counts)
    stats["range_peer_counts_raw_id"] = peer_counts
    stats["range_skipped"] = skipped


def merged_iterator(time_offset, start, end, tracks, stats):
    imu_iter = high_imu_iterator(time_offset, start, end, stats)
    obs_iter = target_observation_iterator(start, end, tracks, stats)

    imu_item = next_or_none(imu_iter)
    obs_item = next_or_none(obs_iter)
    while imu_item is not None or obs_item is not None:
        if obs_item is None or (imu_item is not None and imu_item[2].to_nsec() <= obs_item[2].to_nsec()):
            yield imu_item
            imu_item = next_or_none(imu_iter)
        else:
            yield obs_item
            obs_item = next_or_none(obs_iter)


def main():
    if not HIGH_IMU_BAG.exists():
        raise FileNotFoundError(HIGH_IMU_BAG)
    if not TARGET_BAG.exists():
        raise FileNotFoundError(TARGET_BAG)

    OUT_BAG.parent.mkdir(parents=True, exist_ok=True)
    if OUT_BAG.exists():
        OUT_BAG.unlink()
    if OUT_META.exists():
        OUT_META.unlink()

    time_map = compute_time_map()
    start, end = target_window(time_map)
    tracks = build_anchor_tracks()

    stats = {
        "high_imu_bag": str(HIGH_IMU_BAG),
        "target_observation_bag": str(TARGET_BAG),
        "output_bag": str(OUT_BAG),
        "output_window_start": start,
        "output_window_end": end,
        "output_duration_s": end - start,
        "time_map": time_map,
        "main_uwb_id": MAIN_UWB_ID,
        "peer_to_ekf_id": {str(k): v for k, v in PEER_TO_EKF_ID.items()},
        "peer_to_node": {str(k): v for k, v in PEER_TO_NODE.items()},
        "anchor_track_samples": {str(k): len(v["times"]) for k, v in tracks.items()},
        "notes": [
            f"{TOPIC_IMU} is taken from high precision {TOPIC_IMU} in l型/0611_2.bag.",
            f"{TOPIC_GNSS_ORIGIN}, raw UWB-derived {TOPIC_RANGE}, and {TOPIC_MAG} are taken from 1.156/0611-2.bag.",
            f"{TOPIC_MAG} is extracted from /imu_origin fields 19/20/21 and converted to FRD by flipping Y/Z.",
            f"{TOPIC_RANGE} format follows the 0614 prepared bag: [time, id, lat_rad, lon_rad, h, range, ...].",
            "Anchor positions used inside /range are interpolated from the same 260611 experiment peer node GNSS bags.",
            "If first_gnss_distance_m is large, the high IMU and target observations may not be from the same physical trajectory.",
        ],
    }

    print(f"write output: {OUT_BAG}", flush=True)
    write_progress = Progress("write output", step=5000)
    with rosbag.Bag(str(OUT_BAG), "w", compression=rosbag.Compression.NONE) as outbag:
        for topic, msg, stamp in merged_iterator(time_map["high_ros_to_target_ros"], start, end, tracks, stats):
            outbag.write(topic, msg, stamp)
            write_progress.tick()
    write_progress.close()

    OUT_META.write_text(json.dumps(stats, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(json.dumps(stats, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
