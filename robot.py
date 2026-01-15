import pygame
import math
import json
import time
import random
import os
from dataclasses import dataclass
from typing import List, Optional, Dict, Any

# Initialize Pygame
pygame.init()

# Constants
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (128, 128, 128)

# Physics
PIXELS_PER_METER = 50  # 50 pixels = 1 meter
ROBOT_RADIUS = 0.3  # meters
ROBOT_SPEED = 2.0  # meters per second
ROTATION_SPEED = 2.1  # radians per second (approx 120 deg/s)

@dataclass
class LidarReading:
    angle: float  # Relative angle from LiDAR front (radians)
    abs_angle: float  # Absolute azimuth (0-2pi) like real rotating LiDAR
    hit: bool
    distance: Optional[float]
    lidar_rotation: float  # Current rotation angle of LiDAR unit (0-360°)
    timestamp: float  # Timestamp when this reading was taken
    ring: int  # Ray index (channel) like real multi-beam LiDAR
    sweep_id: int  # Sweep number (increments every 360°)
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert to RPLIDAR A1M8-like format:
        - heading: absolute angle in degrees (0-360°)
        - distance: distance in millimeters (like RPLIDAR)
        - quality: quality level (0-15, higher is better)
        - start_flag: true if this is the first point of a new sweep
        """
        # Check if this is start of new sweep (first ray when lidar_rotation wraps)
        is_start = (self.ring == 0 and self.abs_angle < 1.0) or (self.ring == 0 and self.sweep_id > 0)
        
        return {
            "heading": round(self.abs_angle, 4),  # absolute angle (radians)
            "distance": round(self.distance * 1000, 1) if self.distance is not None else None,  # Convert meters to mm for legacy dict
            "quality": 15 if self.hit else 0,
            "start_flag": is_start,
            # Keep backward compatibility fields
            "angle": round(self.angle, 4),           # relative (radians)
            "abs_angle": round(self.abs_angle, 4),   # absolute azimuth (radians)
            "hit": self.hit,
            "lidar_rotation": round(self.lidar_rotation, 4),
            "timestamp": round(self.timestamp, 3),
            "ring": self.ring,
            "sweep_id": self.sweep_id
        }

@dataclass
class MovementData:
    """Thông tin chuyển động một step của robot.
    
    Updated for SLAM/SAC:
    - delta_distance: distance moved (meters)
    - rotation: rotation command state
    - collision: collision flag
    - dt: time step duration (seconds)
    - linear_velocity: current linear velocity (m/s)
    - angular_velocity: current angular velocity (rad/s)
    - delta_x: change in x (meters)
    - delta_y: change in y (meters)
    - delta_theta: change in angle (radians)
    """
    delta_distance: float
    rotation: str  # "L", "R", "S"
    collision: bool
    dt: float
    linear_velocity: float
    angular_velocity: float
    delta_x: float
    delta_y: float
    delta_theta: float
    
    def to_dict(self) -> Dict[str, Any]: 
        return {
            "delta_distance": round(self.delta_distance, 4),
            "rotation": self.rotation,
            "collision": self.collision,
            "dt": round(self.dt, 4),
            "linear_velocity": round(self.linear_velocity, 3),
            "angular_velocity": round(self.angular_velocity, 3),
            "delta_x": round(self.delta_x, 4),
            "delta_y": round(self.delta_y, 4),
            "delta_theta": round(self.delta_theta, 4)
        }

class Obstacle:
    def __init__(self, x: float, y:  float, width: float, height: float, shape: str = "RECTANGLE"):
        self.x = x  # meters (Center x for CIRCLE, Top-left x for RECTANGLE)
        self.y = y  # meters (Center y for CIRCLE, Top-left y for RECTANGLE)
        self.width = width  # meters (Diameter for CIRCLE)
        self.height = height  # meters (Ignored for CIRCLE)
        self.shape = shape
        
        # For Circle
        self.radius = width / 2.0 if shape == "CIRCLE" else 0.0
        
        self.color = (random.randint(100, 200), random.randint(100, 200), random.randint(100, 200))
    
    def get_rect_pixels(self) -> pygame.Rect:
        if self.shape == "CIRCLE":
            # Bounding box for circle
            cx, cy = self.x * PIXELS_PER_METER, self.y * PIXELS_PER_METER
            r = self.radius * PIXELS_PER_METER
            return pygame.Rect(int(cx - r), int(cy - r), int(r * 2), int(r * 2))
        else:
            return pygame.Rect(
                int(self.x * PIXELS_PER_METER),
                int(self.y * PIXELS_PER_METER),
                int(self.width * PIXELS_PER_METER),
                int(self.height * PIXELS_PER_METER)
            )

    def draw(self, screen: pygame.Surface):
        if self.shape == "CIRCLE":
            center = (int(self.x * PIXELS_PER_METER), int(self.y * PIXELS_PER_METER))
            radius = int(self.radius * PIXELS_PER_METER)
            pygame.draw.circle(screen, self.color, center, radius)
            pygame.draw.circle(screen, BLACK, center, radius, 2)
        else:
            rect = self.get_rect_pixels()
            pygame.draw.rect(screen, self.color, rect)
            pygame.draw.rect(screen, BLACK, rect, 2)
    
    def check_line_intersection(self, x1: float, y1: float, x2: float, y2: float) -> Optional[float]:
        """Check if line intersects with obstacle, return distance if it does"""
        
        if self.shape == "CIRCLE":
            # Line-Circle Intersection
            # Vector d (line direction)
            dx = x2 - x1
            dy = y2 - y1
            
            # Vector f (from line start to circle center)
            fx = x1 - self.x
            fy = y1 - self.y
            
            a = dx*dx + dy*dy
            b = 2 * (fx*dx + fy*dy)
            c = (fx*fx + fy*fy) - self.radius*self.radius
            
            discriminant = b*b - 4*a*c
            
            if discriminant < 0:
                return None
            else:
                discriminant = math.sqrt(discriminant)
                t1 = (-b - discriminant) / (2*a)
                t2 = (-b + discriminant) / (2*a)
                
                # Check valid t in [0, 1]
                t_min = None
                if 0 <= t1 <= 1:
                    t_min = t1
                if 0 <= t2 <= 1:
                    if t_min is None or t2 < t_min:
                        t_min = t2
                
                if t_min is not None:
                    # Intersection point
                    ix = x1 + t_min * dx
                    iy = y1 + t_min * dy
                    return math.sqrt((ix - x1)**2 + (iy - y1)**2)
                return None

        else:
            # RECTANGLE logic
            # Convert obstacle to line segments
            corners = [
                (self.x, self.y),
                (self.x + self.width, self.y),
                (self.x + self.width, self.y + self.height),
                (self.x, self. y + self.height)
            ]
        
        min_distance = None
        
        for i in range(4):
            x3, y3 = corners[i]
            x4, y4 = corners[(i + 1) % 4]
            
            intersection = self._line_intersection(x1, y1, x2, y2, x3, y3, x4, y4)
            if intersection: 
                dist = math.sqrt((intersection[0] - x1)**2 + (intersection[1] - y1)**2)
                if min_distance is None or dist < min_distance:
                    min_distance = dist
        
        return min_distance
    
    @staticmethod
    def _line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
        """Find intersection point of two line segments"""
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        
        if 0 <= t <= 1 and 0 <= u <= 1:
            return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
        return None
    
    def is_colliding_with_circle(self, cx: float, cy: float, radius: float) -> bool:
        """
        Kiểm tra va chạm chính xác bằng toán học (Circle vs Rectangle/Circle).
        Không dùng pixel, dùng tọa độ thực (meters).
        """
        if self.shape == "CIRCLE":
            # Circle-Circle Collision
            dist_sq = (self.x - cx)**2 + (self.y - cy)**2
            combined_radius = self.radius + radius
            return dist_sq <= combined_radius**2
            
        else:
            # RECTANGLE Collision
            # Algorithm:
            # 1. Tìm điểm gần nhất trên rectangle tới tâm đường tròn (Clamp)
            # 2. Tính khoảng cách từ tâm robot tới điểm gần nhất đó
            # 3. Nếu khoảng cách < radius → VA CHẠM
            
            # Tìm điểm gần nhất trên hình chữ nhật tới tâm đường tròn (Clamp)
            closest_x = max(self.x, min(cx, self.x + self.width))
            closest_y = max(self.y, min(cy, self.y + self.height))

        # Tính khoảng cách từ tâm robot tới điểm gần nhất đó
        dx = cx - closest_x
        dy = cy - closest_y
        
        # Nếu khoảng cách nhỏ hơn bán kính robot -> VA CHẠM
        # Dùng squared distance để tránh căn bậc 2 (tối ưu hiệu năng)
        distance_squared = dx * dx + dy * dy
        return distance_squared <= (radius * radius)  # Dùng <= để detect touching

class Robot:
    def __init__(self, x: float, y: float):
        self.x = x  # meters
        self.y = y  # meters
        self.angle = 0.0  # radians
        self.radius = ROBOT_RADIUS  # meters
        
        # Continuous control state (like normal game controls)
        self.keys_pressed = {
            'w': False,  # Move forward
            's': False,  # Move backward
            'a': False,  # Rotate left
            'd': False   # Rotate right
        }
        
        # Movement tracking
        self.last_x = x
        self.last_y = y
        # Movement tracking
        self.last_x = x
        self.last_y = y
        self.last_angle = self.angle  # Fix: init last_angle correctly
        self.total_distance = 0.0
    
    def set_key(self, key: str, pressed: bool):
        """Set key state (pressed/released)"""
        if key in self.keys_pressed:
            self.keys_pressed[key] = pressed
    
    def update(self, dt: float, obstacles: List[Obstacle]):
        """Update robot position based on continuous key states (like normal game)"""
        prev_x, prev_y = self.x, self.y
        prev_angle = self.angle
        
        # 1. Xử lý xoay (Rotation) - Không bao giờ gây va chạm
        rotation_state = "S"
        if self.keys_pressed['a']:
            self.angle -= ROTATION_SPEED * dt
            rotation_state = "L"
        if self.keys_pressed['d']:
            self.angle += ROTATION_SPEED * dt
            rotation_state = "R"
        
        # Normalize angle
        self.angle = (self.angle + math.pi) % (2 * math.pi) - math.pi

        # 2. Tính toán vị trí dự kiến (Predicted Position)
        dx, dy = 0, 0
        if self.keys_pressed['w']:
            dx = math.cos(self.angle) * ROBOT_SPEED * dt
            dy = math.sin(self.angle) * ROBOT_SPEED * dt
        elif self.keys_pressed['s']:
            dx = -math.cos(self.angle) * ROBOT_SPEED * dt
            dy = -math.sin(self.angle) * ROBOT_SPEED * dt
        
        new_x = self.x + dx
        new_y = self.y + dy

        # 3. Kiểm tra va chạm TẠI VỊ TRÍ MỚI
        # Đây là logic "Look-ahead": Nếu đi tới đó mà va chạm thì chặn lại và bật cờ
        is_blocked = self._check_collision_exact(new_x, new_y, obstacles)
        
        # Ngoài ra, kiểm tra xem hiện tại robot có đang bị kẹt không (Overlapping)
        # Trường hợp này hiếm nếu logic chặn tốt, nhưng cần thiết cho SAC để biết robot đang ở trạng thái tồi tệ
        is_stuck = self._check_collision_exact(self.x, self.y, obstacles)

        # Logic di chuyển: Chỉ đi nếu không bị chặn
        if not is_blocked:
            self.x = new_x
            self.y = new_y
        
        # Tính quãng đường
        distance_moved = math.sqrt((self.x - prev_x)**2 + (self.y - prev_y)**2)
        self.total_distance += distance_moved
        
        # Tính toán chi tiết chuyển động cho SLAM/SAC
        actual_delta_x = self.x - prev_x
        actual_delta_y = self.y - prev_y
        
        # Góc thay đổi (radians)
        # Góc thay đổi (radians)
        current_angle_rad = self.angle
        prev_angle_rad = prev_angle
        # Normalize angular difference [-pi, pi]
        delta_theta = math.atan2(math.sin(current_angle_rad - prev_angle_rad), math.cos(current_angle_rad - prev_angle_rad))
        
        # Velocity estimation
        linear_v = distance_moved / dt if dt > 0 else 0.0
        # Angular velocity (chú ý chiều quay)
        # self.angle tăng khi 'd' (turns right, clockwise in typical screen coords but angle decreases? No, wait)
        # Code: 'd' -> angle += ROTATION_SPEED * dt
        # 'a' -> angle -= ROTATION_SPEED * dt
        # So angular_v follows angle change
        angular_v = delta_theta / dt if dt > 0 else 0.0

        # Cờ collision trả về True nếu:
        # 1. Bị chặn khi cố di chuyển (Blocked)
        # 2. Hoặc đang đứng chồng lên vật cản (Stuck)
        collision_flag = is_blocked or is_stuck

        return MovementData(
            delta_distance=distance_moved,
            rotation=rotation_state,
            collision=collision_flag,
            dt=dt,
            linear_velocity=linear_v,
            angular_velocity=angular_v,
            delta_x=actual_delta_x,
            delta_y=actual_delta_y,
            delta_theta=delta_theta
        )
    
    def _check_collision_exact(self, x: float, y: float, obstacles: List[Obstacle]) -> bool:
        """
        Kiểm tra va chạm tuyệt đối dùng tọa độ thực (meters).
        Không dùng pixel, dùng toán học thuần túy.
        
        Returns True nếu robot circle chạm hoặc overlap với bất kỳ obstacle hoặc boundary.
        """
        # 1. Check tường (Boundaries) - Hardcoded theo window size
        # Robot được coi là va chạm nếu mép robot chạm hoặc vượt qua tường
        map_w = WINDOW_WIDTH / PIXELS_PER_METER
        map_h = WINDOW_HEIGHT / PIXELS_PER_METER
        
        if (x - self.radius < 0 or x + self.radius > map_w or
            y - self.radius < 0 or y + self.radius > map_h):
            return True
            
        # 2. Check obstacles bằng toán học
        for obstacle in obstacles:
            if obstacle.is_colliding_with_circle(x, y, self.radius):
                return True
                
        return False
    
    def get_position_pixels(self):
        return (int(self.x * PIXELS_PER_METER), int(self.y * PIXELS_PER_METER))
    
    def get_radius_pixels(self):
        return int(self.radius * PIXELS_PER_METER)

class Lidar:
    def __init__(self, num_rays: int = 6, max_range: float = 3.0,
                 scan_rate: float = 30.0, rotation_speed: float = 34.5):
        """
        Rotating LiDAR giống RPLIDAR A1M8 - 360° Laser Range Scanner
        
        Thông số RPLIDAR A1M8:
        - Rotation speed: 5.5 Hz (330 RPM) = ~34.5 rad/s
        - Range: 0.15m - 3.0m
        
        Thiết kế:
        - scan_rate: 30 Hz (quét mượt, đảm bảo không bỏ sót khi robot di chuyển)
        - rotation_speed: 34.5 rad/s (5.5 Hz)
        - FOV 1.57 rad (90°) với 6 tia, quay độc lập với robot
        """
        self.num_rays = num_rays  # 6 tia trong FOV 90°
        self.max_range = max_range  # meters
        self.scan_rate = scan_rate  # Hz (số lần quét mỗi giây)
        self.scan_interval = 1.0 / scan_rate if scan_rate > 0 else 0.1
        self.rotation_speed = rotation_speed  # radians per second
        self.last_scan_time = 0
        self.last_readings = []
        self.scan_fov = 1.57  # 90° field of view in radians
        
        # LiDAR rotation state
        self.lidar_rotation = 0.0  # Current rotation angle của LiDAR unit (0-360°)
        self.last_update_time = 0
        self.sweep_id = 0
        self._last_rotation_before_mod = 0.0
    
    def scan(self, robot: Robot, obstacles: List[Obstacle], current_time: float) -> Optional[List[LidarReading]]: 
        """
        Perform rotating LiDAR scan với FOV 90° có thể xoay
        - Quét tất cả 6 tia cùng lúc trong FOV 90°
        - LiDAR unit xoay liên tục để quét các hướng khác nhau
        """
        if self.num_rays == 0:
            self.last_readings = []
            return []
        
        # Update LiDAR rotation continuously
        if self.last_update_time > 0:
            dt = current_time - self.last_update_time
            self.lidar_rotation += self.rotation_speed * dt
            # Detect sweep wrap (2pi -> 0) to increment sweep_id
            if self.lidar_rotation >= 2 * math.pi:
                self.lidar_rotation = self.lidar_rotation % (2 * math.pi)
                self.sweep_id += 1
        else:
            dt = 0.0
        
        self.last_update_time = current_time
        
        # Check if it's time to scan
        if current_time - self.last_scan_time < self.scan_interval:
            return None  # Not time to scan yet
        
        self.last_scan_time = current_time
        
        # Calculate ray angles within 90° FOV relative to LiDAR rotation
        angle_step = self.scan_fov / (self.num_rays - 1) if self.num_rays > 1 else 0
        start_angle = -self.scan_fov / 2.0  # -45° relative to LiDAR front
        
        # Scan ALL rays simultaneously within the 90° FOV
        readings = []
        debug_first_ray = True
        
        for ray_idx in range(self.num_rays):
            # Ray angle relative to LiDAR front direction
            ray_angle_relative = start_angle + ray_idx * angle_step
            
            # Absolute angle = LiDAR rotation + ray angle relative to LiDAR (independent of robot angle)
            absolute_angle = (self.lidar_rotation + ray_angle_relative) % (2 * math.pi)
            
            # Calculate ray endpoint
            end_x = robot.x + self.max_range * math.cos(absolute_angle)
            end_y = robot.y + self.max_range * math.sin(absolute_angle)
            
            # Check for intersections
            min_distance = None
            
            for obstacle in obstacles:
                dist = obstacle.check_line_intersection(robot.x, robot.y, end_x, end_y)
                if dist is not None and dist <= self.max_range:
                    if min_distance is None or dist < min_distance:
                        min_distance = dist
            
            # Check window boundaries (ALWAYS has boundary)
            boundary_dist = self._check_boundary_intersection(robot.x, robot.y, end_x, end_y)
            if boundary_dist is not None: 
                if min_distance is None or boundary_dist < min_distance:
                    min_distance = boundary_dist
            
            # Debug first ray
            if debug_first_ray and ray_idx == 0:
                print(f"[LIDAR DEBUG] Ray 0: angle={absolute_angle:.1f}°, endpoint=({end_x:.2f},{end_y:.2f}), dist={min_distance}, boundary={boundary_dist}")
                debug_first_ray = False
            
            # Create reading for this ray
            reading = LidarReading(
                angle=ray_angle_relative,             # Relative to LiDAR front (radians)
                abs_angle=absolute_angle,             # Absolute azimuth (radians)
                hit=min_distance is not None,
                distance=min_distance if min_distance is not None else self.max_range,  # Default to max_range if no hit
                lidar_rotation=self.lidar_rotation,
                timestamp=current_time,
                ring=ray_idx,
                sweep_id=self.sweep_id
            )
            
            readings.append(reading)
        
        # Store and return complete scan
        self.last_readings = readings
        return readings.copy()
    
    
    def to_ros_laserscan_full_360(self, timestamp: Optional[float] = None) -> Dict[str, Any]:
        """
        Convert LiDAR readings to simplified ROS LaserScan format.
        Removed frame_id and intensities as requested for simulation only.
        """
        if timestamp is None:
            timestamp = time.time()
        
        # Current central angle of the scan head
        center_angle_rad = self.lidar_rotation
        
        # FOV extent
        fov_rad = self.scan_fov
        
        # Min/Max angles of this specific single-scan sector
        angle_min = center_angle_rad - (fov_rad / 2.0)
        angle_max = center_angle_rad + (fov_rad / 2.0)
        
        # Calculate increment
        if self.num_rays > 1:
            angle_increment = fov_rad / (self.num_rays - 1)
        else:
            angle_increment = 0.0
            
        ranges = []
        # angles list is useful for debug/verification even if implicit in ROS
        angles = []
        
        sorted_readings = sorted(self.last_readings, key=lambda r: r.ring) if self.last_readings else []

        if not sorted_readings:
             return {
                "timestamp": timestamp,
                "angle_min": angle_min,
                "angle_max": angle_max,
                "angle_increment": angle_increment,
                "time_increment": 0.0,
                "scan_time": self.scan_interval,
                "range_min": 0.15,
                "range_max": self.max_range,
                "ranges": []
            }

        for reading in sorted_readings:
            current_angle = angle_min + (reading.ring * angle_increment)
            angles.append(current_angle)
            
            if reading.hit and reading.distance is not None:
                ranges.append(float(reading.distance))
            else:
                ranges.append(float('inf'))
                
        return {
            "timestamp": timestamp,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "time_increment": 0.0,
            "scan_time": self.scan_interval,
            "range_min": 0.15,
            "range_max": self.max_range,
            "ranges": ranges
        }
    
    def _check_boundary_intersection(self, x1: float, y1: float, x2: float, y2: float) -> Optional[float]: 
        """Check intersection with window boundaries"""
        max_x = WINDOW_WIDTH / PIXELS_PER_METER
        max_y = WINDOW_HEIGHT / PIXELS_PER_METER
        
        boundaries = [
            (0, 0, max_x, 0),      # Top
            (max_x, 0, max_x, max_y),  # Right
            (max_x, max_y, 0, max_y),  # Bottom
            (0, max_y, 0, 0)       # Left
        ]
        
        min_distance = None
        
        for x3, y3, x4, y4 in boundaries:
            intersection = Obstacle._line_intersection(x1, y1, x2, y2, x3, y3, x4, y4)
            if intersection: 
                dist = math.sqrt((intersection[0] - x1)**2 + (intersection[1] - y1)**2)
                if min_distance is None or dist < min_distance:
                    min_distance = dist
        
        return min_distance
    
    def draw(self, screen: pygame.Surface, robot: Robot):
        """Draw lidar rays với FOV 90° có thể xoay"""
        if self.num_rays == 0:
            return
        
        robot_pos = robot.get_position_pixels()
        
        # Draw rays from last scan (independent of robot angle)
        for reading in self.last_readings:
            # Absolute angle stored in reading.abs_angle
            absolute_angle = reading.abs_angle
            
            if reading.hit and reading.distance is not None:
                # Draw to hit point
                end_x = robot.x + reading.distance * math.cos(absolute_angle)
                end_y = robot.y + reading.distance * math.sin(absolute_angle)
                end_pos = (int(end_x * PIXELS_PER_METER), int(end_y * PIXELS_PER_METER))
                pygame.draw.line(screen, RED, robot_pos, end_pos, 2)
                pygame.draw.circle(screen, YELLOW, end_pos, 4)
            else:
                # Draw to max range
                end_x = robot.x + self.max_range * math.cos(absolute_angle)
                end_y = robot.y + self.max_range * math.sin(absolute_angle)
                end_pos = (int(end_x * PIXELS_PER_METER), int(end_y * PIXELS_PER_METER))
                pygame.draw.line(screen, GREEN, robot_pos, end_pos, 1)

class Game:
    def __init__(self, realtime_stream: bool = False, stream_path: str = "realtime_state.jsonl", 
                 enable_sac_control: bool = False, sac_action_file: str = "sac_action.jsonl"):
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("2D Robot Lidar Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        # Game objects
        self.robot = Robot(WINDOW_WIDTH / PIXELS_PER_METER / 2, 
                          WINDOW_HEIGHT / PIXELS_PER_METER / 2)
        # LiDAR mặc định: giống RPLIDAR A1M8 - 6 tia, 90° FOV, quay ?? rad/s, quét 30 Hz, range 3m
        self.lidar = Lidar(num_rays=6, max_range=3.0, scan_rate=30.0, rotation_speed=34.5)
        self.obstacles = self._generate_obstacles()
        
        # State
        self.running = True
        self.start_time = time.time()

        # Realtime streaming (ghi từng dòng JSON để process ngoài đọc realtime)
        self.realtime_stream = realtime_stream
        self.stream_path = stream_path
        self.stream_file = None
        if self.realtime_stream:
            try:
                self.stream_file = open(self.stream_path, "a", buffering=1, encoding="utf-8")
                print(f"[STREAM] Realtime stream to {self.stream_path}")
            except Exception as e:
                print(f"[WARN] Cannot open stream file {self.stream_path}: {e}")
        
        # SAC control (read actions from file)
        self.enable_sac_control = enable_sac_control
        self.sac_action_file = sac_action_file
        self.sac_action_file_handle = None
        self.last_sac_action_time = 0
        if self.enable_sac_control:
            print(f"[SAC Control] Enabled, reading from {self.sac_action_file}")
            try:
                # Open for reading (will seek to end)
                self.sac_action_file_handle = open(self.sac_action_file, 'r')
                # Seek to end to read only new lines
                self.sac_action_file_handle.seek(0, 2)
            except Exception as e:
                print(f"[WARN] Cannot open SAC action file: {e}")
        
        # SLAM (for visualization)
        self.enable_slam_viz = False
        self.slam = None
        self.slam_trajectory = []
    
    def _generate_obstacles(self) -> List[Obstacle]:
        """Generate fixed obstacles for consistent SLAM/SAC evaluation"""
        obstacles = []
        
        # Fixed map với 8 obstacles được thiết kế để test SLAM hiệu quả
        # Layout: Tạo môi trường có hành lang, góc cạnh, và không gian mở
        
        # Tường dọc bên trái
        obstacles.append(Obstacle(2.0, 2.0, 0.8, 4.0))
        
        # Tường ngang trên
        obstacles.append(Obstacle(5.0, 1.5, 5.0, 0.8))
        
        # Chướng ngại vật giữa (hình chữ L)
        obstacles.append(Obstacle(10.0, 5.0, 2.5, 0.8))
        obstacles.append(Obstacle(10.0, 5.0, 0.8, 3.0))
        
        # Tường dọc bên phải
        obstacles.append(Obstacle(18.0, 8.0, 0.8, 5.0))
        
        # Chướng ngại vật nhỏ ở giữa (test collision)
        obstacles.append(Obstacle(12.0, 10.0, 1.2, 1.2))
        
        # Tường ngang dưới
        obstacles.append(Obstacle(3.0, 13.0, 6.0, 0.8))
        
        # Chướng ngại vật góc phải dưới
        obstacles.append(Obstacle(15.0, 12.0, 2.0, 1.5))
        
        # NEW: Circular obstacles
        # Trụ tròn gần tường trên
        obstacles.append(Obstacle(16.0, 3.0, 1.5, 0.0, shape="CIRCLE")) 
        
        # Trụ tròn nhỏ ở góc trái dưới
        obstacles.append(Obstacle(4.0, 10.0, 1.0, 0.0, shape="CIRCLE"))
        
        return obstacles
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                # Continuous control (like normal game)
                if event.key == pygame.K_w:
                    self.robot.set_key('w', True)  # Move forward
                elif event.key == pygame.K_s:
                    self.robot.set_key('s', True)  # Move backward
                elif event.key == pygame.K_a:
                    self.robot.set_key('a', True)  # Rotate left
                elif event.key == pygame.K_d:
                    self.robot.set_key('d', True)  # Rotate right
                elif event.key == pygame.K_r:
                    # Reset
                    self.__init__(self.realtime_stream, self.stream_path)
            elif event.type == pygame.KEYUP:
                # Release keys
                if event.key == pygame.K_w:
                    self.robot.set_key('w', False)
                elif event.key == pygame.K_s:
                    self.robot.set_key('s', False)
                elif event.key == pygame.K_a:
                    self.robot.set_key('a', False)
                elif event.key == pygame.K_d:
                    self.robot.set_key('d', False)
    
    def _read_sac_action(self):
        """Read SAC action from file and apply to robot"""
        if not self.sac_action_file_handle:
            return
        
        try:
            # Read last line
            line = self.sac_action_file_handle.readline()
            if not line:
                return
            
            # Parse JSON
            action_data = json.loads(line.strip())
            linear_vel = action_data.get('linear_velocity', 0.0)
            angular_vel = action_data.get('angular_velocity', 0.0)
            
            # Convert to key states
            # Linear velocity -> forward/backward
            if linear_vel > 0.1:
                self.robot.set_key('w', True)
                self.robot.set_key('s', False)
            elif linear_vel < -0.1:
                self.robot.set_key('s', True)
                self.robot.set_key('w', False)
            else:
                self.robot.set_key('w', False)
                self.robot.set_key('s', False)
            
            # Angular velocity -> left/right rotation
            # angular_vel in rad/s, convert to rotation direction
            if angular_vel > 0.1:  # Turn right
                self.robot.set_key('d', True)
                self.robot.set_key('a', False)
            elif angular_vel < -0.1:  # Turn left
                self.robot.set_key('a', True)
                self.robot.set_key('d', False)
            else:
                self.robot.set_key('a', False)
                self.robot.set_key('d', False)
                
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            # Skip invalid lines
            pass
        except Exception as e:
            # File might be closed or other error
            pass
    
    def update(self, dt: float):
        """Update game state"""
        current_time = time.time() - self.start_time
        
        # Read SAC actions if enabled
        if self.enable_sac_control and self.sac_action_file_handle:
            self._read_sac_action()
        
        # Update robot (processes command queue)
        movement = self.robot.update(dt, self.obstacles)
        
        # Perform lidar scan
        readings = self.lidar.scan(self.robot, self.obstacles, current_time)
        
        # Log if new scan available
        if readings is not None: 
            # Realtime streaming để process ngoài đọc và điều khiển (demo robot thật)
            if self.realtime_stream and self.stream_file:
                # Thêm ROS LaserScan format vào stream
                ros_laserscan = self.lidar.to_ros_laserscan_full_360(
                    timestamp=current_time
                )
                
                # Standardized SLAM/SAC Payload
                stream_payload = {
                    "timestamp": int(current_time * 1000), # ms
                    "dt": movement.dt,
                    "odometry": {
                        "pose": {
                            "x": round(self.robot.x, 4),
                            "y": round(self.robot.y, 4),
                            "theta": round(self.robot.angle, 4)
                        },
                        "velocity": {
                            "linear": movement.linear_velocity,
                            "angular": movement.angular_velocity
                        }
                    },
                    "events": {
                        "collision": movement.collision,
                    },
                    "ros_laserscan": ros_laserscan,
                    "movement": movement.to_dict() # detailed step info
                }
                
                try:
                    self.stream_file.write(json.dumps(stream_payload) + "\n")
                    self.stream_file.flush()
                except Exception as e:
                    print(f"[WARN] Realtime stream write failed: {e}")
    
    def get_state(self):
        """Get current game state for AI (Standardized Observation)"""
        return {
            'timestamp': time.time() - self.start_time,
            'odometry': {
                'x': self.robot.x,
                'y': self.robot.y,
                'theta': self.robot.angle,  # radians
            },
            'lidar_readings': [r.to_dict() for r in self.lidar.last_readings],
            'obstacles': [(o.x, o.y, o.width, o.height) for o in self.obstacles]
        }
    
    def draw(self):
        """Draw everything"""
        self.screen. fill(WHITE)
        
        # Draw obstacles
        for obstacle in self.obstacles:
            obstacle.draw(self.screen)
        
        # Draw lidar rays
        self.lidar.draw(self.screen, self.robot)
        
        # Draw robot
        robot_pos = self.robot.get_position_pixels()
        robot_radius_px = self.robot.get_radius_pixels()
        
        # Color robot differently if AI controlled
        pygame.draw.circle(self.screen, BLUE, robot_pos, robot_radius_px)
        pygame.draw.circle(self.screen, BLACK, robot_pos, robot_radius_px, 2)
        
        # Draw direction indicator
        dir_x = robot_pos[0] + int(robot_radius_px * 1.5 * math.cos(self.robot.angle))
        dir_y = robot_pos[1] + int(robot_radius_px * 1.5 * math.sin(self.robot.angle))
        pygame.draw.line(self.screen, BLACK, robot_pos, (dir_x, dir_y), 3)
        
        # Draw UI
        self._draw_ui()
        
        pygame.display.flip()
    
    def _draw_ui(self):
        """Draw UI elements"""
        # Check if any movement key is pressed
        is_moving = any(self.robot.keys_pressed.values())
        move_status = "MOVING" if is_moving else "STOPPED"
        keys_status = []
        if self.robot.keys_pressed['w']:
            keys_status.append("W")
        if self.robot.keys_pressed['s']:
            keys_status.append("S")
        if self.robot.keys_pressed['a']:
            keys_status.append("A")
        if self.robot.keys_pressed['d']:
            keys_status.append("D")
        keys_str = " ".join(keys_status) if keys_status else "None"
        
        mode_text = "SAC AUTO" if self.enable_sac_control else "MANUAL"
        ui_texts = [
            f"Mode: {mode_text} | W/S: Move forward/backward, A/D: Rotate left/right (hold keys), R: Reset",
            f"Keys Pressed: {keys_str}",
            f"Lidar: {self.lidar.num_rays} rays, {math.degrees(self.lidar.scan_fov):.0f}° FOV (rotating)",
            f"Lidar Range: {self.lidar.max_range:.1f}m",
            f"LiDAR Rotation: {math.degrees(self.lidar.lidar_rotation):.1f}°",
            f"Rotation Speed: {math.degrees(self.lidar.rotation_speed):.1f}°/s",
            f"Robot Position: ({self.robot.x:.2f}, {self.robot.y:.2f})m",
            f"Robot Angle: {math.degrees(self.robot.angle):.1f}°",
            f"Status: {move_status}",
            f"Total Distance: {self.robot.total_distance:.2f}m",
            f"Scan Rate: {self.lidar.scan_rate:.1f} Hz"
        ]
        
        y_offset = 10
        for text in ui_texts:
            surface = self.small_font.render(text, True, BLACK)
            self.screen.blit(surface, (10, y_offset))
            y_offset += 25
    
    def run(self):
        """Main game loop"""
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0  # Delta time in seconds
            
            self.handle_events()
            self.update(dt)
            self.draw()
        
        pygame.quit()

        # Close realtime stream
        if self.stream_file:
            try:
                self.stream_file.close()
            except:
                pass

if __name__ == "__main__":
    import sys
    
    # Parse command line arguments
    enable_stream = os.getenv("ROBOT_REALTIME_STREAM", "1") == "1"
    stream_path = os.getenv("ROBOT_STREAM_PATH", "realtime_state.jsonl")
    enable_sac = "--sac" in sys.argv or os.getenv("ROBOT_ENABLE_SAC", "false").lower() == "true"
    sac_action_file = "sac_action.jsonl"
    
    # Check for custom action file
    if "--sac-action" in sys.argv:
        idx = sys.argv.index("--sac-action")
        if idx + 1 < len(sys.argv):
            sac_action_file = sys.argv[idx + 1]
    
    print(f"[Robot] Stream: {'ON' if enable_stream else 'OFF'}")
    if enable_stream:
        print(f"[Robot] Stream Path: {stream_path}")
    print(f"[Robot] SAC Control: {'ON' if enable_sac else 'OFF'}")
    if enable_sac:
        print(f"[Robot] SAC Action File: {sac_action_file}")
    
    game = Game(realtime_stream=enable_stream, 
                stream_path=stream_path,
                enable_sac_control=enable_sac,
                sac_action_file=sac_action_file)
    game.run()