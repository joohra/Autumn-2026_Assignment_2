#!/usr/bin/env python3
"""
task1_observer.py  — STUDENT TASK 1

Goal: Subscribe to sensor topics and observe raw data.
      Print key values to the console. Identify sensor issues.

Topics to subscribe to:
    /odom        (nav_msgs/Odometry)    — wheel odometry
    /imu_noisy   (sensor_msgs/Imu)      — IMU with injected noise/drift
    /scan_noisy  (sensor_msgs/LaserScan) — LiDAR with injected outliers

What to observe:
    1. What is the robot's position according to odometry after one circle?
       Is it back at (0, 0)? Why or why not?
    2. What does the IMU angular_velocity.z show when the robot is stationary?
       Is it zero? What does a non-zero value mean?
    3. Look at the LiDAR scan ranges. Do any readings look wrong?
       What values indicate a failed/outlier reading?

Questions to answer in your report:
    Q1. What is the odometry position error after 3 full circles?
    Q2. What is the mean and standard deviation of angular_velocity.z
        when the robot is stationary? What does this tell you about the sensor?
    Q3. What percentage of LiDAR rays appear to be outliers?
        How did you identify them?
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import math


class SensorObserver(Node):

    def __init__(self):
        super().__init__("sensor_observer")

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Imu, "/imu_noisy", self.imu_callback, 10)
        self.create_subscription(LaserScan, "/scan_noisy", self.scan_callback, 10)

        # ── State for statistics ──────────────────────────────
        self.imu_gyro_z_samples = []  # collect samples for statistics
        self.scan_count = 0
        self.outlier_count = 0

        # Diy
        self.print_counter = 0

        # Print timer — summary every 5 seconds
        self.create_timer(5.0, self.print_summary)

        self.get_logger().info("SensorObserver started. Listening to sensors...")

    # ─────────────────────────────────────────────────────────
    # TODO 1a: Odometry callback
    # Extract x, y position and yaw from the odometry message.
    # Print them to the console.
    # Hint: position is in msg.pose.pose.position
    #       orientation is a quaternion in msg.pose.pose.orientation
    #       convert quaternion to yaw using:
    #           yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    # ─────────────────────────────────────────────────────────


    def odom_callback(self, msg: Odometry):
        # TODO: extract x, y, yaw and log them

        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        return

        self.print_counter += 1
        if self.print_counter % 20 == 0:
            self.get_logger().info(
                f"Odom -> Pos X: {x_pos:.3f} m, Pos Y: {y_pos:.3f} m | Yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)"
            )


    # ─────────────────────────────────────────────────────────
    # TODO 1b: IMU callback
    # Extract angular_velocity.z (yaw rate) from the IMU message.
    # Store samples in self.imu_gyro_z_samples for statistics.
    # ─────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        angular_velocity = msg.angular_velocity
        z = angular_velocity.z
        self.imu_gyro_z_samples.append(z)

    # ─────────────────────────────────────────────────────────
    # TODO 1c: LiDAR callback
    # Count how many readings are outliers.
    # An outlier is: inf, NaN, or < range_min, or > range_max
    # Also find the minimum valid range (nearest obstacle).
    # ─────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        total_rays = len(msg.ranges)
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min <= r <= msg.range_max
            and not math.isnan(r)
            and not math.isinf(r)
        ]

        outliers_count = total_rays - len(valid_ranges)

        self.scan_count += total_rays
        self.outlier_count += outliers_count


    # ─────────────────────────────────────────────────────────
    # TODO 1d: Print summary statistics
    # Every 5 seconds, print:
    #   - Mean and std dev of IMU gyro Z samples
    #   - Outlier percentage in LiDAR
    # Clear the sample buffer after printing.
    # ─────────────────────────────────────────────────────────
    def print_summary(self):
        # ── 1. IMU Gyro Z Statistics ─────────────────────────────
        if self.imu_gyro_z_samples:
            imu_count = len(self.imu_gyro_z_samples)
            mean_gyro_z = sum(self.imu_gyro_z_samples) / imu_count

            # Calculate sample standard deviation
            variance = sum((x - mean_gyro_z) ** 2 for x in self.imu_gyro_z_samples) / imu_count
            std_gyro_z = math.sqrt(variance)

            self.get_logger().info(
                f"[IMU Summary] Samples: {imu_count} | Mean Gyro Z: {mean_gyro_z:.5f} rad/s | Std Dev: {std_gyro_z:.5f}"
            )
        else:
            self.get_logger().warn("[IMU Summary] No IMU data received in this interval.")

        # ── 2. LiDAR Outlier Statistics ──────────────────────────
        if self.scan_count > 0:
            outlier_percentage = (self.outlier_count / self.scan_count) * 100.0
            self.get_logger().info(
                f"[LiDAR Summary] Total Rays: {self.scan_count} | Outliers: {self.outlier_count} ({outlier_percentage:.2f}%)"
            )
        else:
            self.get_logger().warn("[LiDAR Summary] No LiDAR data received in this interval.")

        # Reset
        self.imu_gyro_z_samples.clear()
        self.scan_count = 0
        self.outlier_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = SensorObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
