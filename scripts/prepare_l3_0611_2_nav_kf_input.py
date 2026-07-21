#!/usr/bin/env python3
import json
import math
from pathlib import Path

import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


IN_BAG = Path("/media/whysad/Lenovo1/260614铜官窑实验/l型/0611_2.bag")
OUT_BAG = Path("/home/whysad/实验数据处理/prepared_bags/l3_0611_2_nav_kf_input.bag")
OUT_META = OUT_BAG.with_suffix(".json")

SRC_IMU_TOPIC = "/zh_origin"
SRC_GNSS_TOPIC = "/L3/gnss_origin"

OUT_IMU_TOPIC = "/zh_origin"
OUT_GNSS_TOPIC = "/gnss_origin"

IMU_RATE_HZ = 200.0
IMU_DT = 1.0 / IMU_RATE_HZ


class Progress:
    def __init__(self, label, total):
        self.label = label
        self.total = int(total)
        self.value = 0
        self.last = 0
        self.step = max(1, self.total // 100) if self.total else 10000

    def tick(self):
        self.value += 1
        if self.value - self.last >= self.step:
            self.last = self.value
            self.render()

    def render(self):
        if self.total:
            pct = self.value * 100.0 / self.total
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


def finite_time(data, stamp):
    if data and math.isfinite(float(data[0])):
        return float(data[0])
    return stamp.to_sec()


def main():
    if not IN_BAG.exists():
        raise FileNotFoundError(IN_BAG)

    OUT_BAG.parent.mkdir(parents=True, exist_ok=True)
    if OUT_BAG.exists():
        OUT_BAG.unlink()
    if OUT_META.exists():
        OUT_META.unlink()

    counts = {
        OUT_IMU_TOPIC: 0,
        OUT_GNSS_TOPIC: 0,
    }
    first_last = {}

    print(f"Input : {IN_BAG}")
    print(f"Output: {OUT_BAG}")
    print("Axis handling: keep /zh_origin unchanged; no Y/Z flip.")

    with rosbag.Bag(str(IN_BAG), "r") as inbag:
        total = inbag.get_message_count(topic_filters=SRC_IMU_TOPIC) + inbag.get_message_count(topic_filters=SRC_GNSS_TOPIC)
        progress = Progress("prepare l3 0611-2", total)

        imu_start_time = None
        imu_count = 0

        with rosbag.Bag(str(OUT_BAG), "w", compression=rosbag.Compression.NONE) as outbag:
            for topic, msg, stamp in inbag.read_messages(topics=[SRC_IMU_TOPIC, SRC_GNSS_TOPIC]):
                progress.tick()
                data = list(msg.data)

                if topic == SRC_IMU_TOPIC:
                    if len(data) < 8:
                        continue
                    if imu_start_time is None:
                        imu_start_time = finite_time(data, stamp)
                    t = imu_start_time + imu_count * IMU_DT
                    out_data = list(data[:8])
                    out_data[0] = t
                    out_data[1] = imu_count * IMU_DT

                    out = make_msg(out_data)
                    outbag.write(OUT_IMU_TOPIC, out, rospy.Time.from_sec(t))
                    counts[OUT_IMU_TOPIC] += 1
                    imu_count += 1
                    first_last.setdefault(OUT_IMU_TOPIC, [t, t])[1] = t
                    continue

                if topic == SRC_GNSS_TOPIC:
                    if len(data) < 7:
                        continue
                    t = finite_time(data, stamp)
                    out = make_msg(data)
                    outbag.write(OUT_GNSS_TOPIC, out, rospy.Time.from_sec(t))
                    counts[OUT_GNSS_TOPIC] += 1
                    first_last.setdefault(OUT_GNSS_TOPIC, [t, t])[1] = t

        progress.close()

    meta = {
        "input_bag": str(IN_BAG),
        "output_bag": str(OUT_BAG),
        "topic_map": {
            SRC_IMU_TOPIC: OUT_IMU_TOPIC,
            SRC_GNSS_TOPIC: OUT_GNSS_TOPIC,
        },
        "topic_counts": counts,
        "first_last_time": first_last,
        "notes": [
            "/zh_origin dtheta/dvel columns are copied unchanged; only data[0] and data[1] are rewritten to a continuous 200Hz time axis.",
            "No frame_index is appended because /zh_origin and /L3/imu_origin frame indices are not reliably one-to-one by timestamp in this source bag.",
            "No magnetometer topic is generated.",
            "No UWB/range topic is generated; this bag is for GNSS/INS only.",
            "The source /zh_origin already has down-axis acceleration increments around -g*dt, so no axis flip is applied.",
        ],
    }
    OUT_META.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(json.dumps(meta, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
