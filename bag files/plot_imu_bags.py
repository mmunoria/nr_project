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

    def read_imu_data(self) -> Dict[str, List[float]]:
        """
        Read IMU data from a ROS 2 bag.

        Returns a dictionary containing:
            time
            linear_acceleration_x/y/z
            angular_velocity_x/y/z
        """
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f"Bag path does not exist: {self.bag_path}")

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path,
            storage_id='mcap'
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

            t_sec = t * 1e-9  # nanoseconds to seconds
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


def plot_signal(ax, time1, y1, time2, y2, time3, y3, title, ylabel):
    ax.plot(time1, y1, label="Before vibration")
    ax.plot(time2, y2, label="During vibration")
    ax.plot(time3, y3, label="After vibration")
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend()


def summarize_signal(name: str, values: List[float]):
    arr = np.array(values)
    print(f"{name}:")
    print(f"  Mean   = {np.mean(arr):.6f}")
    print(f"  Std    = {np.std(arr):.6f}")
    print(f"  Min    = {np.min(arr):.6f}")
    print(f"  Max    = {np.max(arr):.6f}")
    print(f"  RMS    = {np.sqrt(np.mean(arr**2)):.6f}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Read 3 ROS 2 bag files with IMU data and plot vibration-related signals."
    )
    parser.add_argument("--before_bag", required=True, help="Path to bag recorded before vibration")
    parser.add_argument("--during_bag", required=True, help="Path to bag recorded during vibration")
    parser.add_argument("--after_bag", required=True, help="Path to bag recorded after vibration")
    parser.add_argument("--topic", default="/imu", help="IMU topic name, default: /imu")

    args = parser.parse_args()

    before_reader = ImuBagReader(args.before_bag, args.topic)
    during_reader = ImuBagReader(args.during_bag, args.topic)
    after_reader = ImuBagReader(args.after_bag, args.topic)

    before_data = before_reader.read_imu_data()
    during_data = during_reader.read_imu_data()
    after_data = after_reader.read_imu_data()

    print("\n=== Summary: BEFORE vibration ===")
    summarize_signal("linear_acceleration.z", before_data["linear_acceleration_z"])
    summarize_signal("angular_velocity.x", before_data["angular_velocity_x"])
    summarize_signal("angular_velocity.y", before_data["angular_velocity_y"])
    summarize_signal("angular_velocity.z", before_data["angular_velocity_z"])

    print("\n=== Summary: DURING vibration ===")
    summarize_signal("linear_acceleration.z", during_data["linear_acceleration_z"])
    summarize_signal("angular_velocity.x", during_data["angular_velocity_x"])
    summarize_signal("angular_velocity.y", during_data["angular_velocity_y"])
    summarize_signal("angular_velocity.z", during_data["angular_velocity_z"])

    print("\n=== Summary: AFTER vibration ===")
    summarize_signal("linear_acceleration.z", after_data["linear_acceleration_z"])
    summarize_signal("angular_velocity.x", after_data["angular_velocity_x"])
    summarize_signal("angular_velocity.y", after_data["angular_velocity_y"])
    summarize_signal("angular_velocity.z", after_data["angular_velocity_z"])

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle("IMU Comparison: Before, During, and After Vibrating Surface", fontsize=14)

    plot_signal(
        axes[0, 0],
        before_data["time"], before_data["linear_acceleration_x"],
        during_data["time"], during_data["linear_acceleration_x"],
        after_data["time"], after_data["linear_acceleration_x"],
        "Linear Acceleration X", "m/s^2"
    )

    plot_signal(
        axes[0, 1],
        before_data["time"], before_data["linear_acceleration_y"],
        during_data["time"], during_data["linear_acceleration_y"],
        after_data["time"], after_data["linear_acceleration_y"],
        "Linear Acceleration Y", "m/s^2"
    )

    plot_signal(
        axes[1, 0],
        before_data["time"], before_data["linear_acceleration_z"],
        during_data["time"], during_data["linear_acceleration_z"],
        after_data["time"], after_data["linear_acceleration_z"],
        "Linear Acceleration Z", "m/s^2"
    )

    plot_signal(
        axes[1, 1],
        before_data["time"], before_data["angular_velocity_x"],
        during_data["time"], during_data["angular_velocity_x"],
        after_data["time"], after_data["angular_velocity_x"],
        "Angular Velocity X", "rad/s"
    )

    plot_signal(
        axes[2, 0],
        before_data["time"], before_data["angular_velocity_y"],
        during_data["time"], during_data["angular_velocity_y"],
        after_data["time"], after_data["angular_velocity_y"],
        "Angular Velocity Y", "rad/s"
    )

    plot_signal(
        axes[2, 1],
        before_data["time"], before_data["angular_velocity_z"],
        during_data["time"], during_data["angular_velocity_z"],
        after_data["time"], after_data["angular_velocity_z"],
        "Angular Velocity Z", "rad/s"
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()