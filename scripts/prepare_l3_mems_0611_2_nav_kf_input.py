#!/usr/bin/env python3
"""Convert the L3 raw MEMS IMU and GNSS topics to nav_kf input topics.

The source /L3/imu_origin stores angular rate and specific force, while
nav_kf consumes per-sample dtheta and dvel increments.  This converter uses
the frame counter to select the integration interval, so a missing frame
produces an increment over the corresponding longer interval.
"""

import json
import math
from pathlib import Path

import rosbag
import rospy
from std_msgs.msg import Float64MultiArray


IN_BAG = Path("/media/whysad/Lenovo1/260614铜官窑实验/l型/0611_2.bag")
OUT_BAG = Path(
    "/home/whysad/实验数据处理/prepared_bags/"
    "l3_mems_0611_2_nav_kf_input.bag"
)
OUT_META = OUT_BAG.with_suffix(".json")

SRC_IMU_TOPIC = "/L3/imu_origin"
SRC_GNSS_TOPIC = "/L3/gnss_origin"
OUT_IMU_TOPIC = "/zh_origin"
OUT_GNSS_TOPIC = "/gnss_origin"

IMU_RATE_HZ = 200.0
NOMINAL_DT = 1.0 / IMU_RATE_HZ
DEG_TO_RAD = math.pi / 180.0
# This source actually cycles data[14] from 0 through 199 once per second.
# nav_kf itself uses a uint8/256 frame counter, so the converter emits a
# synthetic 0..255 counter after detecting gaps in the source counter.
SOURCE_FRAME_MODULUS = 200


class Progress:
    def __init__(self, label, total):
        self.label = label
        self.total = max(int(total), 1)
        self.value = 0
        self.step = max(1, self.total // 100)
        self.last = -1

    def update(self):
        self.value += 1
        if self.value == self.total or self.value - self.last >= self.step:
            self.last = self.value
            ratio = min(1.0, self.value / self.total)
            print(
                f"\r{self.label}: {self.value}/{self.total} "
                f"({ratio * 100:5.1f}%)",
                end="",
                flush=True,
            )

    def close(self):
        if self.value < self.total:
            self.value = self.total
            self.last = self.total
            print(
                f"\r{self.label}: {self.total}/{self.total} (100.0%)",
                end="",
                flush=True,
            )
        print()


def make_msg(values):
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return msg


def source_time(data, stamp):
    value = float(data[0]) if data else float("nan")
    return value if math.isfinite(value) else stamp.to_sec()


def frame_value(data):
    if len(data) <= 14 or not math.isfinite(float(data[14])):
        return None
    return int(round(float(data[14]))) & 0xFF


def main():
    if not IN_BAG.exists():
        raise FileNotFoundError(IN_BAG)

    OUT_BAG.parent.mkdir(parents=True, exist_ok=True)
    if OUT_BAG.exists():
        OUT_BAG.unlink()
    if OUT_META.exists():
        OUT_META.unlink()

    counts = {OUT_IMU_TOPIC: 0, OUT_GNSS_TOPIC: 0}
    first_last = {}
    stats = {
        "invalid_imu_messages": 0,
        "non_increasing_imu_messages": 0,
        "duplicate_frame_messages": 0,
        "imu_lost_frames": 0,
        "imu_frame_gaps": 0,
        "imu_frame_wraps": 0,
        "imu_dt_min_s": None,
        "imu_dt_max_s": None,
        "imu_dt_mean_s": None,
        "imu_source_stamp_dt_min_s": None,
        "imu_source_stamp_dt_max_s": None,
    }
    dt_values = []
    source_dt_values = []

    with rosbag.Bag(str(IN_BAG), "r") as inbag:
        total = inbag.get_message_count(
            topic_filters=SRC_IMU_TOPIC
        ) + inbag.get_message_count(topic_filters=SRC_GNSS_TOPIC)
        progress = Progress("convert L3 MEMS to nav_kf", total)

        previous = None
        elapsed_start = None
        output_frame = None

        with rosbag.Bag(
            str(OUT_BAG), "w", compression=rosbag.Compression.NONE
        ) as outbag:
            for topic, msg, stamp in inbag.read_messages(
                topics=[SRC_IMU_TOPIC, SRC_GNSS_TOPIC]
            ):
                progress.update()
                data = list(msg.data)

                if topic == SRC_GNSS_TOPIC:
                    if len(data) < 7:
                        continue
                    t = source_time(data, stamp)
                    outbag.write(
                        OUT_GNSS_TOPIC, make_msg(data), rospy.Time.from_sec(t)
                    )
                    counts[OUT_GNSS_TOPIC] += 1
                    first_last.setdefault(OUT_GNSS_TOPIC, [t, t])[1] = t
                    continue

                if len(data) <= 6:
                    stats["invalid_imu_messages"] += 1
                    continue

                t = source_time(data, stamp)
                # An aligned comparison with this bag's /zh_origin gives a
                # Z-axis scale of 57.00 (correlation 0.995), matching 180/pi.
                # The raw fields are therefore deg/s despite being documented
                # elsewhere as rad/s.
                gyro = [
                    float(data[4]) * DEG_TO_RAD,
                    float(data[5]) * DEG_TO_RAD,
                    float(data[6]) * DEG_TO_RAD,
                ]
                accel = [float(data[1]), float(data[2]), float(data[3])]
                if not all(math.isfinite(v) for v in gyro + accel):
                    stats["invalid_imu_messages"] += 1
                    continue

                current_frame = frame_value(data)
                if previous is None:
                    previous = (t, gyro, accel, current_frame)
                    elapsed_start = t
                    continue

                previous_t, previous_gyro, previous_accel, previous_frame = previous
                source_dt = t - previous_t
                if source_dt <= 0.0:
                    stats["non_increasing_imu_messages"] += 1
                    continue
                source_dt_values.append(source_dt)

                frame_delta = None
                if current_frame is not None and previous_frame is not None:
                    frame_delta = (
                        current_frame - previous_frame + SOURCE_FRAME_MODULUS
                    ) % SOURCE_FRAME_MODULUS
                    if frame_delta == 0:
                        stats["duplicate_frame_messages"] += 1
                    elif current_frame < previous_frame:
                        stats["imu_frame_wraps"] += 1

                if frame_delta is not None and frame_delta > 0:
                    dt = frame_delta * NOMINAL_DT
                    if frame_delta > 1:
                        stats["imu_frame_gaps"] += 1
                        stats["imu_lost_frames"] += frame_delta - 1
                else:
                    dt = source_dt

                dtheta = [
                    0.5 * (previous_gyro[i] + gyro[i]) * dt for i in range(3)
                ]
                dvel = [
                    0.5 * (previous_accel[i] + accel[i]) * dt for i in range(3)
                ]
                elapsed = t - elapsed_start
                out_data = [t, elapsed] + dtheta + dvel
                if current_frame is not None:
                    if output_frame is None:
                        output_frame = 0
                    elif frame_delta is not None and frame_delta > 0:
                        output_frame = (output_frame + frame_delta) % 256
                    else:
                        output_frame = (
                            output_frame
                            + max(1, int(round(dt / NOMINAL_DT)))
                        ) % 256
                    out_data.append(float(output_frame))

                outbag.write(
                    OUT_IMU_TOPIC,
                    make_msg(out_data),
                    rospy.Time.from_sec(t),
                )
                counts[OUT_IMU_TOPIC] += 1
                first_last.setdefault(OUT_IMU_TOPIC, [t, t])[1] = t
                first_last[OUT_IMU_TOPIC][0] = min(
                    first_last[OUT_IMU_TOPIC][0], t
                )
                dt_values.append(dt)
                previous = (t, gyro, accel, current_frame)

        progress.close()

    if dt_values:
        stats["imu_dt_min_s"] = min(dt_values)
        stats["imu_dt_max_s"] = max(dt_values)
        stats["imu_dt_mean_s"] = sum(dt_values) / len(dt_values)
    if source_dt_values:
        stats["imu_source_stamp_dt_min_s"] = min(source_dt_values)
        stats["imu_source_stamp_dt_max_s"] = max(source_dt_values)

    meta = {
        "input_bag": str(IN_BAG),
        "output_bag": str(OUT_BAG),
        "topic_map": {
            SRC_IMU_TOPIC: OUT_IMU_TOPIC,
            SRC_GNSS_TOPIC: OUT_GNSS_TOPIC,
        },
        "topic_counts": counts,
        "first_last_time": first_last,
        "integration": {
            "source_accel_fields": "data[1:4], m/s^2",
            "source_gyro_fields": "data[4:7], empirically deg/s",
            "gyro_unit_conversion": "multiply by pi/180 before integration",
            "gyro_unit_evidence": "aligned Z-axis scale versus /zh_origin is 57.00 with correlation 0.995; 180/pi is 57.2958",
            "source_frame_index": "data[14], source counter modulo 200",
            "source_frame_modulus": SOURCE_FRAME_MODULUS,
            "output_frame_index": "synthetic uint8 modulo 256 for nav_kf compatibility",
            "output_format": "[time, elapsed, dtheta_xyz, dvel_xyz, frame_index]",
            "method": "trapezoidal integration of adjacent rate samples",
            "dt_selection": "frame_delta / 200 when frame indices advance; otherwise source timestamp delta",
            "axis_conversion": "none; L3 source axes are kept unchanged",
            "gravity_sign_check": "source stationary z acceleration is approximately -g",
        },
        "stats": stats,
        "notes": [
            "Only /zh_origin and /gnss_origin are written because these are the topics consumed by nav_kf.",
            "The source /L3/imu_origin is rate data; it is converted to dtheta/dvel and is not copied as if it were increments.",
            "The first raw IMU sample is used as the integration start and is not emitted until the next sample arrives.",
            "The source /L3/gnss_origin array is copied unchanged, including data[6] UTC time.",
            "This output is not trimmed to a static alignment segment; use rosbag play --start when selecting the flight segment.",
        ],
    }
    OUT_META.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"Wrote {OUT_BAG}")
    print(f"Wrote {OUT_META}")
    print(json.dumps(meta, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
