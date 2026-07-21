#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import rosbag


IMU_TOPIC = "/zh_origin"
GYRO_COLUMNS = (2, 3, 4)


def main():
    parser = argparse.ArgumentParser(
        description="Scale /zh_origin dtheta columns while preserving all other bag data."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scale", type=float, default=0.5)
    args = parser.parse_args()

    if not args.input.is_file():
        raise FileNotFoundError(args.input)
    if not math.isfinite(args.scale) or args.scale <= 0:
        raise ValueError("--scale must be finite and positive")
    if args.output.exists():
        raise FileExistsError(args.output)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = args.output.with_suffix(".json")
    total = 0
    imu_count = 0
    topic_counts = {}
    next_progress = 5

    with rosbag.Bag(str(args.input), "r") as source:
        total_messages = source.get_message_count()
        with rosbag.Bag(str(args.output), "w", compression=rosbag.Compression.NONE) as target:
            for topic, msg, stamp in source.read_messages():
                if topic == IMU_TOPIC:
                    if len(msg.data) <= max(GYRO_COLUMNS):
                        raise RuntimeError(
                            f"{IMU_TOPIC} message {imu_count} has only {len(msg.data)} fields"
                        )
                    data = list(msg.data)
                    for column in GYRO_COLUMNS:
                        data[column] *= args.scale
                    msg.data = data
                    imu_count += 1

                target.write(topic, msg, stamp)
                topic_counts[topic] = topic_counts.get(topic, 0) + 1
                total += 1

                progress = total * 100 // total_messages
                if progress >= next_progress:
                    print(f"progress: {progress}% ({total}/{total_messages})", flush=True)
                    next_progress += 5

    metadata = {
        "input_bag": str(args.input.resolve()),
        "output_bag": str(args.output.resolve()),
        "operation": {
            "topic": IMU_TOPIC,
            "columns": list(GYRO_COLUMNS),
            "meaning": "dtheta_x, dtheta_y, dtheta_z",
            "scale": args.scale,
        },
        "topic_counts": topic_counts,
        "modified_imu_messages": imu_count,
        "notes": [
            "Only /zh_origin data[2], data[3], and data[4] were scaled.",
            "Accelerometer increments, timestamps, GNSS, range, and magnetometer data are unchanged.",
            "The source bag was preserved.",
        ],
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2), flush=True)


if __name__ == "__main__":
    main()
