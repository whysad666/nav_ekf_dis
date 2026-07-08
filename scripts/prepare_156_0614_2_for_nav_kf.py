#!/usr/bin/env python3
import bisect
import json
import math
from pathlib import Path

import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


ROOT = Path("/media/whysad/Lenovo1/260614铜官窑实验")
BATCH = "0614-2.bag"
OUT_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_nav_kf_input_frd.bag")
OUT_META = OUT_BAG.with_suffix(".json")

TOPIC_IMU = "/zh_origin"
TOPIC_GNSS_ORIGIN = "/gnss_origin"
TOPIC_RANGE = "/range"

MAIN_NODE = "156"
MAIN_UWB_ID = 2

# The current EKF config accepts range IDs 1..12, but not raw UWB id 0.
# Keep the geometry unchanged and only remap the observation tag.
PEER_TO_EKF_ID = {
    0: 4,  # node 154
    1: 1,  # node 153
    3: 3,  # node 155 / RTK base
}

PEER_TO_NODE = {
    0: "154",
    1: "153",
}

# 155 has no RTK/GNSS topic in this batch. This is read from 155/3位置.jpg,
# NMEA GGA: 2827.20353138,N,11248.67678012,E,...,34.7828,M
STATIC_ANCHORS_DEG = {
    3: {
        "node": "155",
        "lat_deg": 28.0 + 27.20353138 / 60.0,
        "lon_deg": 112.0 + 48.67678012 / 60.0,
        "alt_m": 34.7828,
        "source": "155/3位置.jpg GGA first visible fix",
    }
}

IMU_RATE = 200.0
R_EARTH_A = 6378137.0
R_EARTH_E2 = 6.69437999014e-3


class ProgressBar:
    def __init__(self, label, total, width=32):
        self.label = label
        self.total = max(int(total), 0)
        self.width = width
        self.value = 0
        self._last_shown = -1
        self._step = max(1, self.total // 80) if self.total else 1

    def update(self, delta=1):
        self.value += delta
        if self.value == self.total or self.value - self._last_shown >= self._step:
            self._render()

    def _render(self):
        self._last_shown = self.value
        if self.total:
            ratio = min(1.0, self.value / self.total)
            filled = int(self.width * ratio)
            bar = "=" * filled + " " * (self.width - filled)
            print(
                f"\r{self.label} [{bar}] {self.value}/{self.total} {ratio * 100:5.1f}%",
                end="",
                flush=True,
            )
        else:
            print(f"\r{self.label} {self.value}", end="", flush=True)

    def close(self):
        self.value = self.total if self.total else self.value
        self._render()
        print()


def node_bag(node):
    return ROOT / node / BATCH


def radii(lat_rad):
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - R_EARTH_E2 * sin_lat * sin_lat)
    rn = R_EARTH_A / denom
    rm = R_EARTH_A * (1.0 - R_EARTH_E2) / (denom ** 3)
    return rm, rn


def first_uwb_reference(path):
    with rosbag.Bag(str(path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=["/uwb"]):
            data = list(msg.data)
            if len(data) >= 6:
                return {
                    "bag_time": stamp.to_sec(),
                    "msg_time": float(data[0]),
                    "uwb_time": float(data[1]) / 1000.0,
                    "self_id": int(round(data[3])),
                }
    raise RuntimeError(f"no /uwb messages in {path}")


def looks_like_navsatfix(msg):
    return getattr(msg, "_type", "") == "sensor_msgs/NavSatFix" and all(
        hasattr(msg, name) for name in ("latitude", "longitude", "altitude")
    )


def common_time_from_bag_time(ref, bag_time):
    return ref["uwb_time"] + (bag_time - ref["bag_time"])


def load_anchor_track(node, ref):
    path = node_bag(node)
    times = []
    positions = []
    with rosbag.Bag(str(path)) as bag:
        total = bag.get_message_count(topic_filters="/gnss")
        bar = ProgressBar(f"load {node}/gnss", total)
        for _, msg, stamp in bag.read_messages(topics=["/gnss"]):
            if not looks_like_navsatfix(msg):
                bar.update()
                continue
            if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude) or not math.isfinite(msg.altitude):
                bar.update()
                continue
            times.append(common_time_from_bag_time(ref, stamp.to_sec()))
            positions.append((math.radians(msg.latitude), math.radians(msg.longitude), float(msg.altitude)))
            bar.update()
        bar.close()
    if len(times) < 2:
        raise RuntimeError(f"not enough /gnss fixes for node {node}: {len(times)}")
    return {"times": times, "positions": positions}


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


def static_anchor_position(peer_id):
    rec = STATIC_ANCHORS_DEG[peer_id]
    return (math.radians(rec["lat_deg"]), math.radians(rec["lon_deg"]), rec["alt_m"])


def parse_uwb_ranges(data):
    if len(data) < 11:
        return []
    declared_count = int(round(data[5]))
    groups = []
    max_by_len = max(0, (len(data) - 6) // 5)
    for k in range(min(declared_count, max_by_len)):
        base = 6 + 5 * k
        peer_id = int(round(data[base]))
        peer_role = int(round(data[base + 1]))
        range_m = float(data[base + 2])
        groups.append(
            {
                "peer_id": peer_id,
                "peer_role": peer_role,
                "range_m": range_m,
                "extra0": float(data[base + 3]),
                "extra1": float(data[base + 4]),
            }
        )
    return groups


def to_frd(x, y, z):
    # ROS IMU is typically FLU; the EKF here expects FRD.
    return (x, -y, -z)


def write_main_imu(outbag, stats):
    path = node_bag(MAIN_NODE)
    count = 0
    dt = 1.0 / IMU_RATE
    with rosbag.Bag(str(path)) as bag:
        total = bag.get_message_count(topic_filters="/imu")
        bar = ProgressBar("write 156/imu", total)
        for _, msg, stamp in bag.read_messages(topics=["/imu"]):
            t = stamp.to_sec()
            gyr_x, gyr_y, gyr_z = to_frd(
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            )
            acc_x, acc_y, acc_z = to_frd(
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            )
            out = Float64MultiArray()
            out.data = [
                t,
                count * dt,
                gyr_x * dt,
                gyr_y * dt,
                gyr_z * dt,
                acc_x * dt,
                acc_y * dt,
                acc_z * dt,
            ]
            outbag.write(TOPIC_IMU, out, rospy.Time.from_sec(t))
            count += 1
            bar.update()
        bar.close()
    stats["imu_messages"] = count


def write_main_gnss_origin_and_range(outbag, stats, refs, tracks):
    path = node_bag(MAIN_NODE)
    gnss_origin_count = 0
    range_count = 0
    anchor_obs_count = 0
    skipped = {
        "out_of_track": 0,
        "unknown_peer": 0,
        "invalid_range": 0,
        "no_anchor_position": 0,
        "empty_output": 0,
    }
    peer_counts = {}
    main_ref = refs[MAIN_NODE]

    with rosbag.Bag(str(path)) as bag:
        total = bag.get_message_count(topic_filters="/gnss_origin") + bag.get_message_count(topic_filters="/uwb")
        bar = ProgressBar("write 156/gnss_origin+uwb", total)
        for topic, msg, stamp in bag.read_messages(topics=["/gnss_origin", "/uwb"]):
            if topic == "/gnss_origin":
                outbag.write(TOPIC_GNSS_ORIGIN, msg, stamp)
                gnss_origin_count += 1
                bar.update()
                continue

            data = list(msg.data)
            if len(data) < 11:
                bar.update()
                continue
            if int(round(data[3])) != MAIN_UWB_ID:
                bar.update()
                continue

            t_out = float(data[0])
            t_common = float(data[1]) / 1000.0
            if not math.isfinite(t_out):
                t_out = stamp.to_sec()
            out_data = [t_out]

            for obs in parse_uwb_ranges(data):
                peer_id = obs["peer_id"]
                range_m = obs["range_m"]
                if peer_id not in PEER_TO_EKF_ID:
                    skipped["unknown_peer"] += 1
                    continue
                if not math.isfinite(range_m) or range_m < 1.0:
                    skipped["invalid_range"] += 1
                    continue

                if peer_id in STATIC_ANCHORS_DEG:
                    pos = static_anchor_position(peer_id)
                else:
                    track = tracks.get(peer_id)
                    if track is None:
                        skipped["no_anchor_position"] += 1
                        continue
                    pos = interp_position(track, t_common)
                    if pos is None:
                        skipped["out_of_track"] += 1
                        continue

                out_data.extend([float(PEER_TO_EKF_ID[peer_id]), pos[0], pos[1], pos[2], range_m])
                peer_counts[str(peer_id)] = peer_counts.get(str(peer_id), 0) + 1

            if len(out_data) == 1:
                skipped["empty_output"] += 1
                bar.update()
                continue
            out = Float64MultiArray()
            out.data = out_data
            outbag.write(TOPIC_RANGE, out, rospy.Time.from_sec(t_out))
            range_count += 1
            anchor_obs_count += (len(out_data) - 1) // 5
            bar.update()
        bar.close()

    if gnss_origin_count < 2:
        raise RuntimeError(f"not enough main GNSS origin messages: {gnss_origin_count}")
    stats["gnss_origin_messages"] = gnss_origin_count

    stats["range_messages"] = range_count
    stats["range_anchor_observations"] = anchor_obs_count
    stats["range_peer_counts_raw_id"] = peer_counts
    stats["range_skipped"] = skipped
    stats["main_uwb_reference"] = main_ref


def main():
    OUT_BAG.parent.mkdir(parents=True, exist_ok=True)
    if OUT_BAG.exists():
        OUT_BAG.unlink()
    if OUT_META.exists():
        OUT_META.unlink()
    print(f"Output bag: {OUT_BAG}")
    print(f"Output meta: {OUT_META}")

    refs = {}
    for node in ["153", "154", "155", "156"]:
        refs[node] = first_uwb_reference(node_bag(node))

    tracks = {}
    for peer_id, node in PEER_TO_NODE.items():
        tracks[peer_id] = load_anchor_track(node, refs[node])

    stats = {
        "input_root": str(ROOT),
        "batch": BATCH,
        "main_node": MAIN_NODE,
        "main_uwb_id": MAIN_UWB_ID,
        "peer_to_node": {str(k): v for k, v in PEER_TO_NODE.items()},
        "peer_to_ekf_id": {str(k): v for k, v in PEER_TO_EKF_ID.items()},
        "static_anchors_deg": STATIC_ANCHORS_DEG,
        "uwb_references": refs,
        "anchor_track_samples": {
            str(peer): len(track["times"]) for peer, track in tracks.items()
        },
        "notes": [
            f"Output topics are {TOPIC_IMU}, {TOPIC_GNSS_ORIGIN}, {TOPIC_RANGE}.",
            "IMU samples are converted from ROS FLU to FRD by flipping Y and Z.",
            "156 ROS bag time is used for output timestamps.",
            "Other nodes are time-aligned through their first /uwb internal millisecond counter.",
            "Node 155 has no GNSS in this batch; peer id 3 uses a static coordinate read from 155/3位置.jpg.",
            "156 /gnss_origin is copied as-is; no gnss_pv is generated.",
            "Raw UWB peer id 0 is remapped to EKF range id 4 because gi_config.yaml does not accept id 0.",
        ],
    }

    with rosbag.Bag(str(OUT_BAG), "w", compression=rosbag.Compression.NONE) as outbag:
        write_main_imu(outbag, stats)
        write_main_gnss_origin_and_range(outbag, stats, refs, tracks)

    with OUT_META.open("w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2, sort_keys=True)

    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(json.dumps(stats, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
