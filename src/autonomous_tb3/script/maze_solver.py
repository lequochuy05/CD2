#!/usr/bin/env python3
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


MAP_FRAME = "map"


@dataclass(frozen=True)
class InitialPose:
    x: float
    y: float
    yaw_z: float = 0.0 # góc quay Z
    yaw_w: float = 1.0 # góc quay W


DEFAULT_INITIAL_POSE = InitialPose(
    x=-3.915259599685669,
    y=-7.872808456420898,
)


def build_pose_stamped(initial_pose: InitialPose, frame_id: str = MAP_FRAME) -> PoseStamped:
    pose = PoseStamped()   # Tạo PoseStamped để đóng gói dữ liệu vị trí và hướng
    pose.header.frame_id = frame_id  # Gán frame_id là 'map'

    # Gán stamp = 0 để vượt qua bộ lọc kiểm tra dấu thời gian
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0

    pose.pose.position.x = initial_pose.x
    pose.pose.position.y = initial_pose.y
    pose.pose.orientation.z = initial_pose.yaw_z
    pose.pose.orientation.w = initial_pose.yaw_w
    return pose


class MazeSolver:
    """Prepare Nav2 for manual goal selection from RViz."""

    def __init__(self, navigator: BasicNavigator, initial_pose: InitialPose) -> None:
        self.navigator = navigator
        self.initial_pose = initial_pose

    def configure_initial_pose(self) -> None:
        self.navigator.get_logger().info("Dang nap diem xuat phat mac dinh...")
        self.navigator.setInitialPose(build_pose_stamped(self.initial_pose))

    def wait_until_ready(self) -> None:
        self.navigator.get_logger().info("Dang doi Nav2 khoi dong...")
        self.navigator.waitUntilNav2Active()

    def print_ready_message(self) -> None:
        print("==================================================")
        print("Da nap xong diem xuat phat hop le.")
        print("Hay click 'Nav2 Goal' tren RViz va chon diem dich.")
        print("Nav2 se tu tinh duong va dieu khien robot di toi do.")
        print("==================================================")

    def run(self) -> None:
        self.configure_initial_pose()
        self.wait_until_ready()
        self.print_ready_message()


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    try:
        MazeSolver(navigator, DEFAULT_INITIAL_POSE).run()
    except KeyboardInterrupt:
        navigator.get_logger().info("Da dung maze_solver bang Ctrl+C.")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
