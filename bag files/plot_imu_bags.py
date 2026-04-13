#!/usr/bin/env python3

import os
import argparse
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class ImuBagReader:
    def __init__(self, bag_path: str, topic_name: str):
        self.bag_path = bag_path
        self.topic_name = topic_name

    def get_storage_id(self) -> str:
        metadata_file = os.path.join(self.bag_path, "metadata.yaml")

        if not os.path.exists(metadata_file):
            return "mcap"

        with open(metadata_file, "r") as f:
            text = f.read()

        if "storage_identifier: sqlite3" in text:
            return "sqlite3"
        if "storage_identifier: mcap" in text:
            return "mcap"

        return "mcap"

    def read_imu_data(self) -> Dict[str, List[float]]:
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f"Bag path does not exist: {self.bag_path}")

        storage_id = self.get_storage_id()

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path,
            storage_id=storage_id
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        if self.topic_name not in type_map:
            available_topics = "\n".join(type_map.keys())
            raise ValueError(
                f"Topic '{self.topic_name}' not found in bag '{self.bag_path}'.\n"
                f"Available topics:\n{available_topics}"
            )

        msg_type = get_message(type_map[self.topic_name])

        data = {
            "time": [],
            "linear_acceleration_x": [],
            "linear_acceleration_y": [],
            "linear_acceleration_z": [],
            "angular_velocity_x": [],
            "angular_velocity_y": [],
            "angular_velocity_z": [],
        }

        first_time = None

        while reader.has_next():
            topic, serialized_msg, t = reader.read_next()

            if topic != self.topic_name:
                continue

            msg = deserialize_message(serialized_msg, msg_type)

            t_sec = t * 1e-9
            if first_time is None:
                first_time = t_sec

            data["time"].append(t_sec - first_time)
            data["linear_acceleration_x"].append(msg.linear_acceleration.x)
            data["linear_acceleration_y"].append(msg.linear_acceleration.y)
            data["linear_acceleration_z"].append(msg.linear_acceleration.z)
            data["angular_velocity_x"].append(msg.angular_velocity.x)
            data["angular_velocity_y"].append(msg.angular_velocity.y)
            data["angular_velocity_z"].append(msg.angular_velocity.z)

        if len(data["time"]) == 0:
            raise ValueError(
                f"No IMU messages found on topic '{self.topic_name}' in bag '{self.bag_path}'."
            )

        return data


def summarize_signal(name: str, values: List[float]):
    arr = np.array(values)
    print(f"{name}:")
    print(f"  Mean   = {np.mean(arr):.6f}")
    print(f"  Std    = {np.std(arr):.6f}")
    print(f"  Min    = {np.min(arr):.6f}")
    print(f"  Max    = {np.max(arr):.6f}")
    print(f"  RMS    = {np.sqrt(np.mean(arr**2)):.6f}")
    print()


def make_single_plot(
    save_path: str,
    time1, y1,
    time2, y2,
    time3, y3,
    title: str,
    ylabel: str
):
    plt.figure(figsize=(10, 5))
    plt.plot(time1, y1, label="Before vibration")
    plt.plot(time2, y2, label="During vibration")
    plt.plot(time3, y3, label="After vibration")
    plt.title(title)
    plt.xlabel("Time [s]")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Read 3 ROS 2 bag files with IMU data, plot signals, and save each plot."
    )
    parser.add_argument("--before_bag", required=True, help="Path to bag recorded before vibration")
    parser.add_argument("--during_bag", required=True, help="Path to bag recorded during vibration")
    parser.add_argument("--after_bag", required=True, help="Path to bag recorded after vibration")
    parser.add_argument("--topic", default="/imu", help="IMU topic name, default: /imu")
    parser.add_argument(
        "--output_dir",
        default="imu_plots",
        help="Directory to save individual plots"
    )

    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    before_reader = ImuBagReader(args.before_bag, args.topic)
    during_reader = ImuBagReader(args.during_bag, args.topic)
    after_reader = ImuBagReader(args.after_bag, args.topic)

    before_data = before_reader.read_imu_data()
    during_data = during_reader.read_imu_data()
    after_data = after_reader.read_imu_data()

    print("\n=== Summary: BEFORE vibration ===")
    summarize_signal("linear_acceleration.x", before_data["linear_acceleration_x"])
    summarize_signal("linear_acceleration.y", before_data["linear_acceleration_y"])
    summarize_signal("linear_acceleration.z", before_data["linear_acceleration_z"])
    summarize_signal("angular_velocity.x", before_data["angular_velocity_x"])
    summarize_signal("angular_velocity.y", before_data["angular_velocity_y"])
    summarize_signal("angular_velocity.z", before_data["angular_velocity_z"])

    print("\n=== Summary: DURING vibration ===")
    summarize_signal("linear_acceleration.x", during_data["linear_acceleration_x"])
    summarize_signal("linear_acceleration.y", during_data["linear_acceleration_y"])
    summarize_signal("linear_acceleration.z", during_data["linear_acceleration_z"])
    summarize_signal("angular_velocity.x", during_data["angular_velocity_x"])
    summarize_signal("angular_velocity.y", during_data["angular_velocity_y"])
    summarize_signal("angular_velocity.z", during_data["angular_velocity_z"])

    print("\n=== Summary: AFTER vibration ===")
    summarize_signal("linear_acceleration.x", after_data["linear_acceleration_x"])
    summarize_signal("linear_acceleration.y", after_data["linear_acceleration_y"])
    summarize_signal("linear_acceleration.z", after_data["linear_acceleration_z"])
    summarize_signal("angular_velocity.x", after_data["angular_velocity_x"])
    summarize_signal("angular_velocity.y", after_data["angular_velocity_y"])
    summarize_signal("angular_velocity.z", after_data["angular_velocity_z"])

    plot_configs = [
        ("linear_acceleration_x", "Linear Acceleration X", "m/s^2"),
        ("linear_acceleration_y", "Linear Acceleration Y", "m/s^2"),
        ("linear_acceleration_z", "Linear Acceleration Z", "m/s^2"),
        ("angular_velocity_x", "Angular Velocity X", "rad/s"),
        ("angular_velocity_y", "Angular Velocity Y", "rad/s"),
        ("angular_velocity_z", "Angular Velocity Z", "rad/s"),
    ]

    for key, title, ylabel in plot_configs:
        filename = os.path.join(args.output_dir, f"{key}.png")
        make_single_plot(
            filename,
            before_data["time"], before_data[key],
            during_data["time"], during_data[key],
            after_data["time"], after_data[key],
            title,
            ylabel
        )
        print(f"Saved plot: {filename}")


if __name__ == "__main__":
    main()