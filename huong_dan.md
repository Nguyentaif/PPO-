# Hướng Dẫn Chi Tiết Simulation Robot
Tài liệu này cung cấp thông tin kỹ thuật đầy đủ về simulation robot, bao gồm thông số kỹ thuật, định dạng dữ liệu, và hướng dẫn tích hợp cho SLAM và Reinforcement Learning (RL).

## 1. Thông Tin Robot & Môi Trường
### Robot Base
- **Kích thước**: Bán kính 0.3m (đường kính 0.6m).
- **Vận tốc tuyến tính (v)**: Tối đa 2.0 m/s.
- **Vận tốc góc (ω)**: Tối đa 120 độ/giây (~2.09 rad/s).
- **Điều khiển**: Mô hình Differential Drive đơn giản hóa (Non-holonomic).

### LiDAR Sensor
Mô phỏng RPLIDAR A1M8 quay 360 độ.
- **Cấu hình tia**: 6 tia quét đồng thời trong vùng nhìn (FOV) 1.57 radians (90 độ).
- **Tốc độ quay (Rotation Speed)**: ~34.5 radians/giây (5.5 Hz).
- **Tốc độ quét (Scan Rate)**: 30 Hz (30 lần cập nhật/giây).
- **Tầm xa (Range)**: 0.15m - 3.0m.
- **Cơ chế**: LiDAR quay liên tục độc lập với hướng robot. Dữ liệu đầu ra là **Sector Scan** (một phần 360 độ), cần được ghép lại bởi thuật toán SLAM.

### Môi trường
- **Tỷ lệ (Scale)**: 50 pixels = 1 mét.
- **Kích thước**: 1200x800 pixels (tương đương 24m x 16m).
- **Vật cản**: 
    - Tường bao quanh map.
    - Chướng ngại vật hình chữ nhật (tường, block).
    - Chướng ngại vật hình tròn (trụ, cột).

## 2. Thông Tin Output (Realtime Data Stream)
Simulation stream dữ liệu thời gian thực ra file [realtime_state.jsonl](file:///c:/K20/AmazingTech_Proj/realtime_state.jsonl). Mỗi dòng là một JSON object đại diện cho một bước thời gian (tick).

### Cấu Trúc JSON
```json
{
  "timestamp": 1234,              // (ms) - Milliseconds từ lúc bắt đầu
  "dt": 0.016,                    // (s) - Delta time (giây) của step này
  "odometry": {
    "pose": {
      "x": 10.5,                  // (m) - Tọa độ X (mét)
      "y": 5.2,                   // (m) - Tọa độ Y (mét)
      "theta": 0.7853             // (rad) - Hướng robot (radians)
    },
    "velocity": {
      "linear": 1.5,              // (m/s) - Vận tốc dài hiện tại
      "angular": 0.5              // (rad/s) - Vận tốc góc hiện tại
    }
  },
  "events": {
    "collision": false            // True nếu va chạm với chướng ngại vật
  },
  "ros_laserscan": {              // Format LaserScan đơn giản hóa (cho simulation/algorithm dev)
    "timestamp": 1234.56,         // (s) - Timestamp của scan
    "angle_min": 0.0,             // Góc bắt đầu sector quét (radians)
    "angle_max": 1.57,            // Góc kết thúc sector quét (radians)
    "angle_increment": 0.314,     // Bước góc giữa các tia (radians)
    "range_min": 0.15,            // (m) - Mét
    "range_max": 3.0,             // (m) - Mét
    "ranges": [2.5, 2.6, ...]     // Mảng khoảng cách (mét). Infinity nếu không có vật cản.
  },
  "movement": {                   // Thông tin chi tiết step di chuyển (dùng cho debug/RL reward)
    "delta_distance": 0.02,       // (m) - Quãng đường di chuyển
    "rotation": "S",              // Trạng thái quay: L (Trái), R (Phải), S (Dừng)
    "delta_x": 0.015,             // (m)
    "delta_y": 0.012,             // (m)
    "delta_theta": 0.005          // (rad)
  }
}
```

## 3. Hướng Dẫn Tích Hợp SLAM
Để xây dựng bản đồ (Mapping) và định vị (Localization):

1.  **Đọc Stream**: Mở file [realtime_state.jsonl](file:///c:/K20/AmazingTech_Proj/realtime_state.jsonl) và đọc từng dòng mới nhất (kiểu `tail -f`).
2.  **Lấy Dữ Liệu**:
    *   **Odometry**: Sử dụng `odometry.pose` (`x`, `y`, `theta_rad`) làm thông tin ước lượng vị trí (Robot Pose Guess).
    *   **Laser Scan**: Sử dụng object [ros_laserscan](file:///c:/K20/AmazingTech_Proj/robot.py#443-513).
        *   Đây là **Partial Scan** (scan từng phần). Thuật toán SLAM (như GMapping, HectorSLAM) cần tích hợp các scan này theo thời gian.
        *   `angle_min`, `angle_max` là góc tuyệt đối của sector đang quét *tại thời điểm đó*.
3.  **Lưu ý Coordinates**:
    *   Hệ tọa độ Robot: X hướng tới trước, Y hướng sang trái (chuẩn ROS).
    *   Hệ tọa độ Map: Gốc (0,0) ở góc trên-trái (chuẩn Pygame/Image), nhưng data output là mét.
    *   Cần transform: `Map_X = X_meter`, `Map_Y = Y_meter`, `Angle = Theta`.

## 4. Hướng Dẫn Tích Hợp Reinforcement Learning (RL/SAC)
Để huấn luyện agent:

1.  **State Space (Observation)**:
    *   Dùng hàm `game.get_state()` hoặc parse từ JSON log:
    *   **Robot Pose**: `odometry.pose.x`, `odometry.pose.y` (đơn vị: mét), `theta` (radians).
    *   **Lidar**: Lấy mảng `ranges` từ [ros_laserscan](file:///c:/K20/AmazingTech_Proj/robot.py#443-513) (đơn vị: mét). Chuẩn hóa về [0, 1] bằng cách chia cho `range_max` (3.0). Thay thế `Infinity` bằng 1.0.
    *   **Velocity**: `odometry.velocity.linear` (m/s), `odometry.velocity.angular` (rad/s).
    *   **Goal**: (Cần define thêm target point bên ngoài và tính khoảng cách/góc tới đích).

2.  **Reward Function**:
    *   (+) Di chuyển về phía mục tiêu (`movement.delta_distance` hướng về target).
    *   (-) Va chạm: Phạt nặng nếu `events.collision == true`.
    *   (-) Quay vòng tròn quá nhiều mà không đi (phạt `angular_velocity` lớn).

3.  **Action Space**:
    *   Continuous: `[linear_velocity, angular_velocity]`
    *   Range: `linear` [0, 2.0], `angular` [-2.09, 2.09].

4. Chuyển Mode Điều Khiển
Để chuyển quyền điều khiển từ bàn phím (Manual) sang AI (RL):

### Cách 1: Qua Argument (Khởi động)
Chạy command line với cờ `--sac`:
```powershell
python robot.py --sac
```
Hoặc dùng file action tùy chỉnh:
```powershell
python robot.py --sac --sac-action my_actions.jsonl
```

### Cách 2: Qua Biến Môi Trường (Environment Variable)
Set biến môi trường trước khi chạy:
*   Windows Powershell: `$env:ROBOT_ENABLE_SAC="true"; python robot.py`
*   CMD: `set ROBOT_ENABLE_SAC=true && python robot.py`

### Giao thức điều khiển SAC
Khi mode SAC bật, robot sẽ đọc file `sac_action.jsonl` (hoặc file chỉ định) liên tục để lấy lệnh điều khiển.
Format file action (mỗi dòng 1 JSON):
```json
{"linear": 1.5, "angular": -0.5}
```
