#!/usr/bin/env python3
"""
task2_filter.py  — STUDENT TASK 2

Goal: Clean up noisy sensor data using simple filters.
      Publish cleaned versions of each sensor topic.

Input topics:
    /imu_noisy   (sensor_msgs/Imu)        — IMU with constant gyro bias + noise
    /scan_noisy  (sensor_msgs/LaserScan)  — LiDAR with outlier rays

Output topics you must publish:
    /imu_filtered   (sensor_msgs/Imu)        — cleaned IMU
    /scan_filtered  (sensor_msgs/LaserScan)  — cleaned LiDAR

Filters to implement:

    IMU — Running average on angular_velocity.z
        Keep a running mean that updates with each new reading:
            self.gyro_z_avg = 0.9 * self.gyro_z_avg + 0.1 * new_reading
        This smooths out random noise (the 0.1 factor controls how fast it adapts).
        Note: this does NOT remove the constant bias — only reduces random noise.

    LiDAR — Invalid ray removal
        Replace any ray that is inf or below range_min with 0.0.
        A value of 0.0 signals "no valid reading here".

Questions to answer in your report:
    Q1. Compare angular_velocity.z in /imu_noisy vs /imu_filtered.
        Does the running average remove the constant offset (bias)?
        Why or why not?
    Q2. After filtering the LiDAR, what fraction of rays are now 0.0?
        Are these the same rays you identified as invalid in Task 1?
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
import math


class SensorFilter(Node):

    def __init__(self):
        super().__init__('sensor_filter')

        # Running average state for gyro Z
        self.gyro_z_avg = 0.0

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(
            Imu, '/imu_noisy', self.imu_callback, 10)
        self.create_subscription(
            LaserScan, '/scan_noisy', self.scan_callback, 10)

        # ── Publishers ────────────────────────────────────────
        self.imu_pub  = self.create_publisher(Imu, '/imu_filtered', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        self.get_logger().info(
            'SensorFilter started.\n'
            '  /imu_noisy   → /imu_filtered\n'
            '  /scan_noisy  → /scan_filtered'
        )

    # ─────────────────────────────────────────────────────────
    # TODO 2a: IMU running average filter
    #
    # 1. Update the running average:
    #      self.gyro_z_avg = 0.9 * self.gyro_z_avg + 0.1 * msg.angular_velocity.z
    #
    # 2. Create a new Imu message, copy the original, and replace
    #    angular_velocity.z with self.gyro_z_avg.
    #
    # 3. Publish the filtered message on /imu_filtered.
    #
    # Hint — how to copy and modify a message:
    #   filtered = Imu()
    #   filtered.header              = msg.header
    #   filtered.angular_velocity    = msg.angular_velocity
    #   filtered.linear_acceleration = msg.linear_acceleration
    #   filtered.angular_velocity.z  = self.gyro_z_avg   # override z only
    # ─────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        # 1. Update the running average (Low-pass filter)
        self.gyro_z_avg = 0.9 * self.gyro_z_avg + 0.1 * msg.angular_velocity.z

        # 2. Create a new Imu message and copy original header/data
        filtered = Imu()
        filtered.header = msg.header
        filtered.orientation = msg.orientation
        filtered.orientation_covariance = msg.orientation_covariance
        filtered.angular_velocity = msg.angular_velocity
        filtered.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered.linear_acceleration = msg.linear_acceleration
        filtered.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Override angular_velocity.z with the filtered value
        filtered.angular_velocity.z = self.gyro_z_avg

        # 3. Publish the filtered message on /imu_filtered
        self.imu_pub.publish(filtered)


    # ─────────────────────────────────────────────────────────
    # TODO 2b: LiDAR invalid ray removal
    #
    # 1. Copy the incoming LaserScan into a new message.
    # 2. Go through each value in msg.ranges:
    #    - If it is inf or below msg.range_min → replace with 0.0
    #    - Otherwise keep the original value
    # 3. Publish the cleaned scan on /scan_filtered.
    #
    # Hint:
    #   ranges = list(msg.ranges)   # make a mutable copy
    #   for i in range(len(ranges)):
    #       if math.isinf(ranges[i]) or ranges[i] < msg.range_min:
    #           ranges[i] = 0.0
    # ─────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        # 1. Create a new LaserScan message and copy metadata
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.intensities = msg.intensities

        # 2. Make a mutable list copy of the tuple msg.ranges and clean invalid rays
        ranges = list(msg.ranges)
        for i in range(len(ranges)):
            # Replace inf, NaN, or values below range_min with 0.0
            if math.isinf(ranges[i]) or math.isnan(ranges[i]) or ranges[i] < msg.range_min:
                ranges[i] = 0.0

        # Assign cleaned ranges to the filtered message
        filtered_scan.ranges = ranges

        # 3. Publish the cleaned scan on /scan_filtered
        self.scan_pub.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()