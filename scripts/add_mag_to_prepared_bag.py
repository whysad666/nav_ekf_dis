#!/usr/bin/env python3
import json
import math
import shutil
from pathlib import Path

import rosbag
from std_msgs.msg import Float64MultiArray


SOURCE_BAG = Path("/media/whysad/Lenovo1/260614铜官窑实验/156/0614-2.bag")
TARGET_BAG = Path("/home/whysad/实验数据处理/prepared_bags/156_0614-2_nav_kf_input.bag")
TARGET_META = TARGET_BAG.with_suffix(".json")
BACKUP_BAG = TARGET_BAG.with_name(f"{TARGET_BAG.stem}_pre_mag.bag")
TMP_BAG = TARGET_BAG.with_name(f"{TARGET_BAG.stem}.tmp_mag.bag")

SOURCE_TOPIC = "/imu_origin"
MAG_TOPIC = "/mag_origin"


class Progress:
    def __init__(self, label, total):
        self.label = label
        self.total = int(total)
        self.value = 0
        self.step = max(1, self.total // 100) if self.total else 1
        self.last = -1

    def tick(self):
        self.value += 1
        if self.value == self.total or self.value - self.last >= self.step:
            self.last = self.value
            if self.total:
                print(
                    f"\r{self.label}: {self.value}/{self.total} "
                    f"({self.value * 100.0 / self.total:5.1f}%)",
                    end="",
                    flush=True,
                )

    def close(self):
        if self.total:
            self.value = self.total
            self.tick()
        print()


def make_mag_message(imu_origin_msg, stamp):
    data = list(imu_origin_msg.data)
    if len(data) <= 21:
        return None
    mag_x, mag_y, mag_z = data[19], data[20], data[21]
    if not all(math.isfinite(v) for v in (mag_x, mag_y, mag_z)):
        return None

    out = Float64MultiArray()
    # Source imu_origin uses the same raw body frame as /imu. The prepared
    # /zh_origin topic is FRD, so flip Y/Z here to keep both sensors aligned.
    out.data = [stamp.to_sec(), mag_x, -mag_y, -mag_z]
    return out


def next_or_none(iterator):
    try:
        return next(iterator)
    except StopIteration:
        return None


def mag_iterator(source_bag):
    for _, msg, stamp in source_bag.read_messages(topics=[SOURCE_TOPIC]):
        out = make_mag_message(msg, stamp)
        if out is not None:
            yield MAG_TOPIC, out, stamp


def update_meta(mag_count):
    if not TARGET_META.exists():
        return
    with TARGET_META.open("r", encoding="utf-8") as f:
        meta = json.load(f)
    meta["mag_origin_messages"] = mag_count
    meta["mag_origin_topic"] = MAG_TOPIC
    notes = meta.setdefault("notes", [])
    note = (
        f"{MAG_TOPIC} is extracted from 156 {SOURCE_TOPIC} fields 19/20/21 "
        "and converted to FRD by flipping Y/Z; units are microtesla."
    )
    if note not in notes:
        notes.append(note)
    with TARGET_META.open("w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2, sort_keys=True)


def main():
    if not SOURCE_BAG.exists():
        raise FileNotFoundError(SOURCE_BAG)
    if not TARGET_BAG.exists():
        raise FileNotFoundError(TARGET_BAG)

    if TMP_BAG.exists():
        TMP_BAG.unlink()

    if not BACKUP_BAG.exists():
        shutil.copy2(TARGET_BAG, BACKUP_BAG)
        print(f"Backup: {BACKUP_BAG}")
    else:
        print(f"Backup already exists: {BACKUP_BAG}")

    with rosbag.Bag(str(TARGET_BAG), "r") as target_in, rosbag.Bag(str(SOURCE_BAG), "r") as source_in:
        target_count = target_in.get_message_count()
        source_info = source_in.get_type_and_topic_info().topics
        source_count = source_info[SOURCE_TOPIC].message_count
        total = target_count + source_count
        progress = Progress("merge mag topic", total)

        with rosbag.Bag(str(TMP_BAG), "w", compression=rosbag.Compression.NONE) as outbag:
            target_iter = target_in.read_messages()
            mag_iter = mag_iterator(source_in)

            target_item = next_or_none(target_iter)
            mag_item = next_or_none(mag_iter)
            mag_count = 0

            while target_item is not None or mag_item is not None:
                write_mag = False
                if target_item is None:
                    write_mag = True
                elif mag_item is not None and mag_item[2].to_nsec() < target_item[2].to_nsec():
                    write_mag = True

                if write_mag:
                    topic, msg, stamp = mag_item
                    outbag.write(topic, msg, stamp)
                    mag_count += 1
                    mag_item = next_or_none(mag_iter)
                    progress.tick()
                    continue

                topic, msg, stamp = target_item
                if topic != MAG_TOPIC:
                    outbag.write(topic, msg, stamp)
                target_item = next_or_none(target_iter)
                progress.tick()

        progress.close()

    TMP_BAG.replace(TARGET_BAG)
    update_meta(mag_count)
    print(f"Wrote: {TARGET_BAG}")
    print(f"Added {MAG_TOPIC}: {mag_count} messages")


if __name__ == "__main__":
    main()
