# Mô phỏng giải mê cung tự động bằng Turtlebot3

Kho lưu trữ này chứa mã để tạo ra một **mô phỏng Gazebo Classic** của robot **Turtlebot3** (***waffle***) (được thiết kế bởi *Robotis*) tự động di chuyển qua một mê cung đơn giản.

Kho lưu trữ này được xây dựng dựa trên kiến ​​thức và hướng dẫn của khóa học **Udemy** **ROS2 Autonomous Driving and SLAM using NAV2 with TurtleBot3** do **Muhammad Luqman** tạo ra.

Các công cụ phần mềm chủ yếu được sử dụng để xây dựng dự án này là:

- Trình thông dịch Python3
- Khung ROS2 cơ bản
- Trình mô phỏng Gazebo Classic
- Gói ROS2 *turtlebot3_gazebo*

- Gói ROS2 *slam_toolbox*

- Ngăn xếp Navigation2

Hệ điều hành:

- Ubuntu (*bạn có thể sử dụng phiên bản mới nhất hiện có*)

Để triển khai dự án này thành công, tất cả các phần mềm được liệt kê ở trên phải được cài đặt trên hệ điều hành Ubuntu của bạn.

## Điểm KHÁC BIỆT trong Dự án này?

Ban đầu, dự án này được xây dựng như một gói ROS2 loại **ament_python** trong Khóa học đã đề cập ở trên. Tuy nhiên, trong kho lưu trữ này, tôi đã xây dựng dự án như một gói ROS2 loại **ament_cmake**.

## Triển khai

Để triển khai dự án này, vui lòng làm theo các bước sau:

- **Tạo một thư mục mới** ở một vị trí thích hợp trong hệ điều hành Ubuntu của bạn. Bạn có thể đặt tên thư mục bất kỳ. Nhưng để minh họa, tôi đặt tên là **Cloned Repo**

- Mở một **cửa sổ terminal mới** bên trong thư mục đó.

- **Sao chép kho lưu trữ này** vào thư mục bằng cách chạy lệnh sau trong terminal:

```bash

git clone https://github.com/lequochuy05/CD2.git

git clone https://github.com/lequochuy05/CD2.git

```

- Thao tác này sẽ tạo một thư mục mới có tên **CD2** bên trong thư mục **Cloned Repo**.

- Thao tác này sẽ tạo một thư mục mới có tên **CD2** bên trong thư mục **Cloned Repo**.

- Truy cập vào thư mục **CD2** thông qua cửa sổ terminal đã mở trước đó.

```

cd CD2/

```

- Tiếp theo, chúng ta cần biên dịch dự án này. Vì vậy, hãy chạy lệnh sau từ cùng một cửa sổ terminal.

```bash

colcon build

```

Thao tác này sẽ tạo ra 3 (ba) thư mục mới bên trong thư mục **CD2**, đó là - **build**, **install** và **log**.

- Đóng cửa sổ terminal trước đó.

- Cuối cùng, chúng ta có thể triển khai dự án.

- Mở một cửa sổ terminal mới bên trong thư mục **CD2** và chạy các lệnh sau:

```bash

source install/setup.bash

ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py

```

Thao tác này sẽ mở một **cửa sổ mô phỏng Gazebo Classic** (với thế giới mê cung mô phỏng và một robot ***Turtlebot3*** bên trong) VÀ một **cửa sổ RViz2** (nơi bạn có thể thấy bản đồ 2D của thế giới mê cung).

Sử dụng ***con lăn chuột*** để ***phóng to/thu nhỏ*** trong cả môi trường **Gazebo** và **RViz2**.

Sử dụng **nút chuột trái** (để di chuyển) và ***con lăn chuột*** (để xoay) -- để điều chỉnh vị trí và góc nhìn của **thế giới mê cung** trong môi trường **Gazebo** - theo ý muốn của bạn.

Sử dụng **nút chuột trái** (để XOAY) và ***nút cuộn chuột*** (để DI CHUYỂN) -- để điều chỉnh vị trí và góc nhìn của **bản đồ mê cung 2D** trong môi trường **RViz2** - sao cho thuận tiện nhất.

Ngoài ra, trước khi tiến hành bước tiếp theo, bạn nên mở cả hai cửa sổ **Gazebo** và **Rviz2** cạnh nhau để có thể quan sát những gì đang diễn ra trong cả hai cửa sổ cùng một lúc.

- Mở một cửa sổ terminal song song thứ hai bên trong thư mục **CD2** (trong khi vẫn giữ cửa sổ terminal trước đó hoạt động) và chạy lệnh sau từ đó:

```bash

source install/setup.bash

ros2 run autonomous_tb3 maze_solver.py
```