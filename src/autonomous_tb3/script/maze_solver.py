#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # 1. THIẾT LẬP ĐIỂM XUẤT PHÁT MẶC ĐỊNH (Hardcoded Initial Pose)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    # Gán thời gian = 0 để tránh lỗi lệch thời gian giữa hệ thống (Wall-time) và Gazebo (Sim-time)
    initial_pose.header.stamp.sec = 0
    initial_pose.header.stamp.nanosec = 0
    
    initial_pose.pose.position.x = -3.915259599685669
    initial_pose.pose.position.y = -7.872808456420898
    
    # SỬA LỖI QUATERNION: (z=0.2, w=0.9999...) trong code cũ là sai mặt toán học (không chuẩn hóa = 1).
    # Điều này khiến AMCL báo lỗi "malformed" và từ chối nhận vị trí.
    # Sửa lại thành góc 0 độ (z=0.0, w=1.0) là hợp lệ.
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    print("Đang nạp điểm xuất phát mặc định...")
    navigator.setInitialPose(initial_pose)

    # Đợi hệ thống Nav2 khởi động xong
    navigator.waitUntilNav2Active()

    print("==================================================")
    print("✅ Đã nạp xong điểm xuất phát hợp lệ!")
    print("🎯 Bây giờ hãy click nút 'Nav2 Goal' trên RViz")
    print("   và chọn điểm đích, xe sẽ tự chạy tới đó.")
    print("==================================================")

    # QUAN TRỌNG: KHÔNG GÁN CỨNG ĐIỂM ĐÍCH Ở ĐÂY NỮA
    # Hệ thống Nav2 sẽ tự động nhận điểm đích từ công cụ "Nav2 Goal" trên RViz.

    # Chỉ tắt node Python này, KHÔNG tắt hệ thống Nav2
    navigator.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()