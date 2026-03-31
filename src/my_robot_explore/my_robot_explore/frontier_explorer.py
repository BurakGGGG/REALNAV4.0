#!/usr/bin/env python3
"""
Advanced Frontier Explorer for Autonomous Exploration
Teknofest Sanayi Dijital Teknolojiler - İleri Seviye Yarışması

Özellikler:
- Information Gain based frontier selection
- Frontier clustering for better goal selection
- Stuck detection and recovery
- Real-time exploration metrics
- Visualization with MarkerArray
- Performance optimizations (ROI-based scanning)
"""

import math
from typing import Optional, Tuple, List, Set, Dict
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, Header, ColorRGBA

import tf2_ros


def neighbors4(x: int, y: int):
    """4-connected neighbors"""
    yield x + 1, y
    yield x - 1, y
    yield x, y + 1
    yield x, y - 1


def neighbors8(x: int, y: int):
    """8-connected neighbors for clustering"""
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            yield x + dx, y + dy


class FrontierCluster:
    """Frontier cluster representation"""
    def __init__(self, x: int, y: int):
        self.cells: List[Tuple[int, int]] = [(x, y)]
        self.center_x = x
        self.center_y = y
        self.size = 1
        self.information_gain = 0.0  # Will be calculated
    
    def add_cell(self, x: int, y: int):
        self.cells.append((x, y))
        self.size = len(self.cells)
        # Update center (simple average)
        self.center_x = round(sum(c[0] for c in self.cells) / self.size)
        self.center_y = round(sum(c[1] for c in self.cells) / self.size)
    
    def contains(self, x: int, y: int) -> bool:
        return (x, y) in self.cells


class FrontierExplorer(Node):
    """
    Advanced Frontier Explorer with:
    - Information gain based selection
    - Frontier clustering
    - Stuck detection
    - Metrics and visualization
    """

    def __init__(self):
        super().__init__("frontier_explorer")

        # Params
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("nav_action", "navigate_to_pose")

        self.declare_parameter("min_goal_distance", 0.5)
        self.declare_parameter("goal_cooldown_sec", 0.5)
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("return_to_start_timeout_sec", 120.0)  # Return-to-start için daha uzun timeout
        self.declare_parameter("replan_period_sec", 1.5)

        self.declare_parameter("free_value_max", 10)
        self.declare_parameter("occupied_value_min", 65)
        self.declare_parameter("safety_cell_radius", 3)
        
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("start_position_timeout", 10.0)
        self.declare_parameter("start_position_tolerance", 0.6)  # Nav2 xy_goal_tolerance (0.5) + margin
        
        # New parameters for advanced features
        self.declare_parameter("use_information_gain", True)
        self.declare_parameter("information_gain_weight", 2.0)  # Weight for information gain vs distance
        self.declare_parameter("frontier_cluster_distance", 5)  # Pixels - max distance for clustering
        self.declare_parameter("min_cluster_size", 3)  # Minimum cells in a cluster
        
        self.declare_parameter("stuck_detection_enabled", True)
        self.declare_parameter("stuck_distance_threshold", 0.5)  # meters
        self.declare_parameter("stuck_time_threshold", 30.0)  # seconds
        self.declare_parameter("stuck_velocity_threshold", 0.05)  # m/s - velocity-based stuck detection
        self.declare_parameter("use_velocity_stuck_detection", True)  # Enable velocity-based stuck detection
        
        self.declare_parameter("roi_radius", 15.0)  # meters - Region of Interest radius (base)
        self.declare_parameter("roi_radius_min", 10.0)  # meters - Minimum ROI radius
        self.declare_parameter("roi_radius_max", 20.0)  # meters - Maximum ROI radius
        self.declare_parameter("use_dynamic_roi", True)  # Enable dynamic ROI based on coverage
        self.declare_parameter("publish_visualization", True)
        self.declare_parameter("publish_metrics", True)
        
        self.declare_parameter("visualization_topic", "/frontier_markers")
        self.declare_parameter("metrics_topic_prefix", "/exploration_metrics")

        # Get parameters
        self.map_topic = self.get_parameter("map_topic").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.nav_action = self.get_parameter("nav_action").value

        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.goal_cooldown = float(self.get_parameter("goal_cooldown_sec").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.return_to_start_timeout = float(self.get_parameter("return_to_start_timeout_sec").value)
        self.replan_period = float(self.get_parameter("replan_period_sec").value)

        self.free_value_max = int(self.get_parameter("free_value_max").value)
        self.occupied_value_min = int(self.get_parameter("occupied_value_min").value)
        self.safety_cell_radius = int(self.get_parameter("safety_cell_radius").value)
        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        self.start_position_timeout = float(self.get_parameter("start_position_timeout").value)
        self.start_position_tolerance = float(self.get_parameter("start_position_tolerance").value)
        
        self.use_information_gain = bool(self.get_parameter("use_information_gain").value)
        self.information_gain_weight = float(self.get_parameter("information_gain_weight").value)
        self.frontier_cluster_distance = int(self.get_parameter("frontier_cluster_distance").value)
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        
        self.stuck_detection_enabled = bool(self.get_parameter("stuck_detection_enabled").value)
        self.stuck_distance_threshold = float(self.get_parameter("stuck_distance_threshold").value)
        self.stuck_time_threshold = float(self.get_parameter("stuck_time_threshold").value)
        self.stuck_velocity_threshold = float(self.get_parameter("stuck_velocity_threshold").value)
        self.use_velocity_stuck_detection = bool(self.get_parameter("use_velocity_stuck_detection").value)
        
        self.roi_radius_base = float(self.get_parameter("roi_radius").value)
        self.roi_radius_min = float(self.get_parameter("roi_radius_min").value)
        self.roi_radius_max = float(self.get_parameter("roi_radius_max").value)
        self.use_dynamic_roi = bool(self.get_parameter("use_dynamic_roi").value)
        self.roi_radius = self.roi_radius_base  # Current ROI radius (can be dynamic)
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)
        self.publish_metrics = bool(self.get_parameter("publish_metrics").value)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 action
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

        # Map
        self.map_msg: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        
        # Odometry for robot velocity and heading
        self.odom_msg: Optional[Odometry] = None
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        # KRİTİK: Mission complete olduğunda robotu durdurmak için cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Visualization
        if self.publish_visualization:
            self.marker_pub = self.create_publisher(
                MarkerArray, self.get_parameter("visualization_topic").value, 10
            )

        # Metrics publishers
        if self.publish_metrics:
            metrics_prefix = self.get_parameter("metrics_topic_prefix").value
            self.metrics_pubs = {
                'coverage': self.create_publisher(Float32, f"{metrics_prefix}/coverage", 10),
                'frontier_count': self.create_publisher(Float32, f"{metrics_prefix}/frontier_count", 10),
                'total_distance': self.create_publisher(Float32, f"{metrics_prefix}/total_distance", 10),
                'exploration_time': self.create_publisher(Float32, f"{metrics_prefix}/exploration_time", 10),
            }

        # State
        self.active_goal_handle = None
        self.active_goal_started = None
        self.last_goal_time = self.get_clock().now()
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self._last_goal_distance: float = float('inf')  # Son goal mesafesi (akıllı goal değiştirme için)
        self._last_goal_distance_check_time = None  # Son kontrol zamanı
        self.start_position: Optional[Tuple[float, float]] = None
        self.start_position_saved = False
        self.start_time = self.get_clock().now()
        self.exploration_complete = False
        self.returning_to_start = False
        self.return_to_start_attempts = 0  # Return-to-start deneme sayısı
        self.max_return_attempts = 5  # Maksimum deneme sayısı
        self.mission_complete = False  # Mission tamamen tamamlandı mı?
        
        # Stuck detection
        self.robot_positions = deque(maxlen=50)  # Store last 50 positions
        self.last_significant_movement = self.get_clock().now()
        self.failed_goals: deque = deque(maxlen=20)  # Track failed goals (FIFO - oldest removed first)
        self.robot_velocities = deque(maxlen=20)  # Store recent velocities for velocity-based stuck detection
        
        # Metrics
        self.total_distance = 0.0
        self.last_robot_xy: Optional[Tuple[float, float]] = None
        self.last_metrics_publish = self.get_clock().now()
        
        # Cached frontiers
        self.last_frontier_scan_time = None
        self.cached_frontier_clusters: List[FrontierCluster] = []
        
        # Robot state tracking
        self.robot_yaw: float = 0.0
        self.robot_velocity: Tuple[float, float] = (0.0, 0.0)  # (linear, angular)
        
        # Goal history learning: track successful vs failed regions
        self.successful_goals: Dict[Tuple[int, int], int] = {}  # (cell_x, cell_y) -> success_count
        self.failed_goals_history: Dict[Tuple[int, int], int] = {}  # (cell_x, cell_y) -> fail_count
        self.goal_history_radius = 2.0  # meters - region size for history tracking
        
        # Exploration efficiency tracking
        self.exploration_start_time = self.get_clock().now()
        self.last_coverage = 0.0
        self.coverage_history = deque(maxlen=20)  # Track coverage over time
        self.exploration_rate = 0.0  # Coverage per second
        
        # Path cost cache (Nav2 path cost estimation)
        self.path_cost_cache: Dict[Tuple[float, float], float] = {}  # (gx, gy) -> path_cost
        self.path_cost_cache_time: Dict[Tuple[float, float], float] = {}  # Cache timestamp
        self.path_cost_cache_ttl = 5.0  # seconds - cache TTL
        
        # Predictive exploration: frontier future value estimation
        self.frontier_age: Dict[Tuple[int, int], float] = {}  # Track how long frontier exists (max 500)

        self.timer = self.create_timer(self.replan_period, self._tick)

        # Metrics timer (publish every 2 seconds)
        if self.publish_metrics:
            self.metrics_timer = self.create_timer(2.0, self._publish_metrics)

        self.get_logger().info("Advanced FrontierExplorer started with:")
        self.get_logger().info(f"  - Information Gain: {self.use_information_gain}")
        self.get_logger().info(f"  - Stuck Detection: {self.stuck_detection_enabled}")
        self.get_logger().info(f"  - Visualization: {self.publish_visualization}")
        self.get_logger().info(f"  - Metrics: {self.publish_metrics}")

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        # Invalidate cache when map updates
        self.cached_frontier_clusters = []
    
    def _odom_cb(self, msg: Odometry):
        """Track robot velocity and heading for smarter goal selection"""
        self.odom_msg = msg
        # Extract yaw from orientation (quaternion to euler)
        orientation_q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        # Track velocity
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        self.robot_velocity = (linear_vel, angular_vel)
        # Store velocity for stuck detection
        if self.use_velocity_stuck_detection:
            self.robot_velocities.append((linear_vel, angular_vel))

    def _get_robot_xy_in_map(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.robot_frame, rclpy.time.Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f"TF not ready ({self.goal_frame}->{self.robot_frame}): {e}")
            return None
    
    def _get_robot_yaw_in_map(self) -> Optional[float]:
        """Get robot yaw in map frame"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.robot_frame, rclpy.time.Time()
            )
            orientation_q = tf.transform.rotation
            # Convert quaternion to yaw
            siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
            cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return yaw
        except Exception as e:
            return None

    def _cell_is_safe(self, data, w, h, cx, cy) -> bool:
        """Check if cell is safe (no obstacles in radius)"""
        r = self.safety_cell_radius
        for yy in range(cy - r, cy + r + 1):
            for xx in range(cx - r, cx + r + 1):
                if 0 <= xx < w and 0 <= yy < h:
                    v = data[yy * w + xx]
                    if v >= self.occupied_value_min:
                        return False
        return True

    def _calculate_information_gain(self, data, w, h, cx, cy, radius_cells: int = 5) -> float:
        """Calculate information gain: number of unknown cells in radius"""
        unknown_count = 0
        for yy in range(cy - radius_cells, cy + radius_cells + 1):
            for xx in range(cx - radius_cells, cx + radius_cells + 1):
                if 0 <= xx < w and 0 <= yy < h:
                    idx = yy * w + xx
                    if data[idx] == -1:  # Unknown
                        unknown_count += 1
        return float(unknown_count)

    def _is_frontier_cell(self, data, w, h, x, y) -> bool:
        """Check if cell is a frontier (free cell with unknown neighbor)"""
        idx = y * w + x
        v = data[idx]
        if v == -1 or v > self.free_value_max:
            return False
        
        # Check if has unknown neighbor
        for nx, ny in neighbors4(x, y):
            if 0 <= nx < w and 0 <= ny < h:
                if data[ny * w + nx] == -1:
                    return True
        return False

    def _cluster_frontiers(self, frontier_cells: List[Tuple[int, int]]) -> List[FrontierCluster]:
        """Cluster nearby frontier cells together"""
        if not frontier_cells:
            return []
        
        clusters: List[FrontierCluster] = []
        used = set()
        
        for fx, fy in frontier_cells:
            if (fx, fy) in used:
                continue
            
            # Start new cluster
            cluster = FrontierCluster(fx, fy)
            used.add((fx, fy))
            
            # BFS to find nearby cells
            queue = deque([(fx, fy)])
            while queue:
                cx, cy = queue.popleft()
                for nx, ny in neighbors8(cx, cy):
                    if (nx, ny) in used:
                        continue
                    if (nx, ny) not in frontier_cells:
                        continue
                    # Check distance from cluster center
                    dist = math.sqrt((nx - cluster.center_x)**2 + (ny - cluster.center_y)**2)
                    if dist <= self.frontier_cluster_distance:
                        cluster.add_cell(nx, ny)
                        used.add((nx, ny))
                        queue.append((nx, ny))
            
            if cluster.size >= self.min_cluster_size:
                clusters.append(cluster)
        
        return clusters

    def _update_dynamic_roi(self):
        """Update ROI radius based on exploration coverage"""
        if not self.use_dynamic_roi:
            self.roi_radius = self.roi_radius_base
            return
        
        coverage = self._calculate_coverage()
        
        # Early exploration: smaller ROI (more focused)
        # Mid exploration: medium ROI
        # Late exploration: larger ROI (find remaining frontiers)
        if coverage < 30.0:
            # Early stage: smaller ROI for focused exploration
            self.roi_radius = self.roi_radius_min
        elif coverage < 70.0:
            # Mid stage: base ROI
            self.roi_radius = self.roi_radius_base
        else:
            # Late stage: larger ROI to find remaining frontiers
            self.roi_radius = self.roi_radius_max
    
    def _find_frontier_clusters(self, robot_xy: Tuple[float, float]) -> List[FrontierCluster]:
        """Find and cluster frontiers with ROI optimization"""
        m = self.map_msg
        if m is None:
            return []

        w = m.info.width
        h = m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data

        def idx(x, y): return y * w + x

        rx, ry = robot_xy

        # Update dynamic ROI based on coverage
        self._update_dynamic_roi()

        # ROI: Only scan within radius
        roi_cells = int(self.roi_radius / res)
        robot_cell_x = int((rx - ox) / res)
        robot_cell_y = int((ry - oy) / res)
        
        # Bounds for ROI
        min_x = max(1, robot_cell_x - roi_cells)
        max_x = min(w - 1, robot_cell_x + roi_cells)
        min_y = max(1, robot_cell_y - roi_cells)
        max_y = min(h - 1, robot_cell_y + roi_cells)

        frontier_cells = []
        for y in range(min_y, max_y):
            for x in range(min_x, max_x):
                if not self._is_frontier_cell(data, w, h, x, y):
                    continue
                if not self._cell_is_safe(data, w, h, x, y):
                    continue
                frontier_cells.append((x, y))

        # Cluster frontiers
        clusters = self._cluster_frontiers(frontier_cells)
        
        # Calculate information gain for each cluster
        for cluster in clusters:
            cluster.information_gain = self._calculate_information_gain(
                data, w, h, cluster.center_x, cluster.center_y
            )
        
        # Predictive exploration: Eski frontier'leri temizle (artık yoksa)
        current_time = self.get_clock().now().nanoseconds / 1e9
        cluster_keys = set()
        for cluster in clusters:
            gx = ox + (cluster.center_x + 0.5) * res
            gy = oy + (cluster.center_y + 0.5) * res
            cx_cell = int((gx - ox) / res)
            cy_cell = int((gy - oy) / res)
            cluster_keys.add((cx_cell, cy_cell))
        
        # Artık var olmayan frontier'leri temizle
        keys_to_remove = [k for k in self.frontier_age.keys() if k not in cluster_keys]
        for key in keys_to_remove:
            del self.frontier_age[key]

        return clusters

    def _pick_frontier_goal(self, robot_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Select best frontier goal using information gain"""
        m = self.map_msg
        if m is None:
            return None

        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        rx, ry = robot_xy

        # Find frontier clusters (use cache if recent)
        current_time = self.get_clock().now()
        use_cache = (self.last_frontier_scan_time is not None and
                    (current_time - self.last_frontier_scan_time).nanoseconds / 1e9 < 1.0)
        
        if not use_cache or not self.cached_frontier_clusters:
            self.cached_frontier_clusters = self._find_frontier_clusters(robot_xy)
            self.last_frontier_scan_time = current_time

        clusters = self.cached_frontier_clusters

        if not clusters:
            return None

        best_cluster = None
        best_score = -1e9

        for cluster in clusters:
            # Convert cluster center to world coordinates
            gx = ox + (cluster.center_x + 0.5) * res
            gy = oy + (cluster.center_y + 0.5) * res

            # Distance to robot
            dist = math.hypot(gx - rx, gy - ry)
            if dist < self.min_goal_distance:
                continue

            # Check if this goal was recently failed (deque - FIFO)
            # Check if goal is close to any failed goal (within 0.5m)
            goal_failed = False
            for failed_goal in self.failed_goals:
                if math.hypot(gx - failed_goal[0], gy - failed_goal[1]) < 0.5:
                    goal_failed = True
                    break
            if goal_failed:
                continue

            # Check if too close to last goal (prevent going to same area)
            if self.last_goal_xy is not None:
                if math.hypot(gx - self.last_goal_xy[0], gy - self.last_goal_xy[1]) < 2.0:  # Artırıldı: 1.0 → 2.0 (daha geniş alan)
                    continue

            # ============================================================
            # AKILLI MULTI-CRITERIA SCORING SİSTEMİ
            # ============================================================
            
            # 1. Information Gain (Bilgi Kazancı)
            if self.use_information_gain:
                info_score = (cluster.information_gain * self.information_gain_weight) / (dist + 1.0)
                # Büyük kümeler bonus (daha fazla keşfedilmemiş alan)
                info_score += cluster.size * 0.15
            else:
                info_score = 0.0
            
            # 2. Distance Penalty (Mesafe cezası - yakın hedefler tercih edilir)
            distance_score = -dist * 0.3  # Negatif: uzak = kötü
            
            # 3. Obstacle Density Penalty (Engel yoğunluğu cezası)
            # Goal etrafındaki engel yoğunluğunu hesapla
            obstacle_density = self._calculate_obstacle_density(gx, gy, robot_xy)
            obstacle_penalty = -obstacle_density * 0.5  # Çok engelli = kötü
            
            # 4. Coverage Potential (Keşif potansiyeli)
            # Goal'ın etrafındaki keşfedilmemiş alan potansiyeli
            coverage_potential = self._calculate_coverage_potential(gx, gy, robot_xy)
            coverage_bonus = coverage_potential * 0.4  # Yüksek potansiyel = iyi
            
            # 5. Frontier Accessibility (Erişilebilirlik)
            # Goal'a erişim kolaylığı (obstacle-free path olasılığı)
            accessibility = self._calculate_accessibility(gx, gy, robot_xy)
            accessibility_bonus = accessibility * 0.3
            
            # Toplam skor
            score = info_score + distance_score + obstacle_penalty + coverage_bonus + accessibility_bonus
            
            # 6. Adaptive Weighting (Coverage'a göre ağırlık ayarı)
            coverage = self._calculate_coverage()
            if coverage < 30.0:
                # Erken aşama: Information gain'e daha fazla önem ver
                score = score * 0.7 + info_score * 0.3
            elif coverage > 80.0:
                # Geç aşama: Distance'a daha fazla önem ver (kalan alanları bul)
                score = score * 0.6 + distance_score * 0.4
            
            # 6b. Adaptive Exploration: Coverage-based parameter adjustment
            # Coverage'a göre parametreleri dinamik olarak ayarla
            adaptive_bonus = self._get_adaptive_exploration_bonus(coverage, dist, cluster.size)
            score += adaptive_bonus
            
            # 7. Frontier Quality Assessment: accessibility and shape analysis
            frontier_quality = self._assess_frontier_quality(cluster, gx, gy, robot_xy)
            score += frontier_quality * 0.5  # Quality bonus
            
            # 8. Goal History Learning: track successful vs failed regions
            history_bonus = self._get_goal_history_bonus(gx, gy, robot_xy)
            score += history_bonus
            
            # 9. Path Cost Estimation: Nav2 path cost'u kullan (daha kolay yolları tercih et)
            path_cost_penalty = self._estimate_path_cost(gx, gy, robot_xy)
            score -= path_cost_penalty * 0.3  # Yüksek cost = ceza
            
            # 10. Predictive Exploration: Frontier'in gelecekteki değerini tahmin et
            predictive_bonus = self._predict_frontier_value(cluster, gx, gy, robot_xy)
            score += predictive_bonus
            
            # 11. Exploration Efficiency: Coverage rate'e göre optimizasyon
            efficiency_bonus = self._get_efficiency_bonus(coverage, dist)
            score += efficiency_bonus
            
            # 7. Yön Bonusu (Robotun önündeki hedefler tercih edilir) - ÇOK DAHA AGRESİF!
            robot_yaw = self._get_robot_yaw_in_map()
            if robot_yaw is not None:
                goal_yaw = math.atan2(gy - ry, gx - rx)
                yaw_diff = abs(math.atan2(math.sin(goal_yaw - robot_yaw), 
                                         math.cos(goal_yaw - robot_yaw)))
                # Robotun önündeki hedefler: ÇOK BÜYÜK BONUS (çok daha az dönüş!)
                if yaw_diff < math.pi / 4:  # 45 degrees (daha dar açı)
                    score *= 2.0  # 100% bonus for forward goals (1.6 → 2.0)
                elif yaw_diff < math.pi / 3:  # 60 degrees
                    score *= 1.5  # 50% bonus for slightly forward goals
                # Robotun arkasındaki hedefler: ÇOK BÜYÜK CEZA (U-turn kesinlikle istemiyoruz!)
                elif yaw_diff > 2 * math.pi / 3:  # > 120 degrees (behind robot)
                    score *= 0.2  # 80% penalty for backward goals (0.4 → 0.2)
                elif yaw_diff > math.pi / 2:  # > 90 degrees (sideways)
                    score *= 0.6  # 40% penalty for sideways goals

            if score > best_score:
                best_score = score
                best_cluster = cluster

        if best_cluster is None:
            return None

        # Convert to world coordinates
        gx = ox + (best_cluster.center_x + 0.5) * res
        gy = oy + (best_cluster.center_y + 0.5) * res

        return (gx, gy)

    def _calculate_obstacle_density(self, gx: float, gy: float, robot_xy: Tuple[float, float]) -> float:
        """
        Goal etrafındaki engel yoğunluğunu hesapla (0.0-1.0)
        Yüksek değer = çok engelli bölge (kötü)
        """
        if self.map_msg is None:
            return 0.0
        
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data
        w = m.info.width
        h = m.info.height
        
        # Goal'ı map koordinatına çevir
        gx_cell = int((gx - ox) / res)
        gy_cell = int((gy - oy) / res)
        
        # Goal etrafında 2m radius içindeki engel sayısını say
        radius_cells = int(2.0 / res)  # 2m radius
        obstacle_count = 0
        total_cells = 0
        
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = gx_cell + dx
                y = gy_cell + dy
                if 0 <= x < w and 0 <= y < h:
                    idx = y * w + x
                    total_cells += 1
                    if data[idx] >= self.occupied_value_min:  # Occupied
                        obstacle_count += 1
        
        if total_cells == 0:
            return 0.0
        
        return float(obstacle_count) / float(total_cells)
    
    def _calculate_coverage_potential(self, gx: float, gy: float, robot_xy: Tuple[float, float]) -> float:
        """
        Goal'ın etrafındaki keşfedilmemiş alan potansiyelini hesapla (0.0-1.0)
        Yüksek değer = çok keşfedilmemiş alan (iyi)
        """
        if self.map_msg is None:
            return 0.0
        
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data
        w = m.info.width
        h = m.info.height
        
        # Goal'ı map koordinatına çevir
        gx_cell = int((gx - ox) / res)
        gy_cell = int((gy - oy) / res)
        
        # Goal etrafında 3m radius içindeki unknown sayısını say
        radius_cells = int(3.0 / res)  # 3m radius
        unknown_count = 0
        total_cells = 0
        
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = gx_cell + dx
                y = gy_cell + dy
                if 0 <= x < w and 0 <= y < h:
                    idx = y * w + x
                    total_cells += 1
                    if data[idx] == -1:  # Unknown
                        unknown_count += 1
        
        if total_cells == 0:
            return 0.0
        
        return float(unknown_count) / float(total_cells)
    
    def _calculate_accessibility(self, gx: float, gy: float, robot_xy: Tuple[float, float]) -> float:
        """
        Goal'a erişilebilirlik skoru (0.0-1.0)
        Robot'tan goal'a giden yol üzerindeki engel yoğunluğunu değerlendir
        Yüksek değer = erişilebilir (iyi)
        """
        if self.map_msg is None:
            return 1.0
        
        rx, ry = robot_xy
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data
        w = m.info.width
        h = m.info.height
        
        # Robot ve goal'ı map koordinatına çevir
        rx_cell = int((rx - ox) / res)
        ry_cell = int((ry - oy) / res)
        gx_cell = int((gx - ox) / res)
        gy_cell = int((gy - oy) / res)
        
        # Robot'tan goal'a doğru yol üzerindeki hücreleri kontrol et
        # Bresenham line algorithm kullanarak yol üzerindeki hücreleri al
        steps = max(abs(gx_cell - rx_cell), abs(gy_cell - ry_cell))
        if steps == 0:
            return 1.0
        
        obstacle_count = 0
        for i in range(steps + 1):
            t = float(i) / float(steps)
            x = int(rx_cell + t * (gx_cell - rx_cell))
            y = int(ry_cell + t * (gy_cell - ry_cell))
            
            if 0 <= x < w and 0 <= y < h:
                idx = y * w + x
                if data[idx] >= self.occupied_value_min:  # Occupied
                    obstacle_count += 1
        
        # Erişilebilirlik = 1.0 - (obstacle_ratio)
        obstacle_ratio = float(obstacle_count) / float(steps + 1)
        return max(0.0, 1.0 - obstacle_ratio * 2.0)  # 2x penalty for obstacles

    def _get_adaptive_exploration_bonus(self, coverage: float, dist: float, cluster_size: int) -> float:
        """
        Adaptive Exploration: Coverage-based parameter adjustment
        Coverage'a göre parametreleri dinamik olarak ayarla
        """
        bonus = 0.0
        
        # Erken aşama (<30%): Büyük kümeleri tercih et, uzak hedeflerden kaçın
        if coverage < 30.0:
            if cluster_size > 10:  # Büyük kümeler
                bonus += 2.0
            if dist < 5.0:  # Yakın hedefler
                bonus += 1.0
            elif dist > 10.0:  # Uzak hedefler
                bonus -= 1.0
        
        # Orta aşama (30-70%): Dengeli yaklaşım
        elif coverage < 70.0:
            if cluster_size > 5:  # Orta-büyük kümeler
                bonus += 1.0
            if 3.0 < dist < 8.0:  # Orta mesafe hedefler
                bonus += 0.5
        
        # Geç aşama (>70%): Küçük kümeleri de değerlendir, uzak hedeflere git
        else:
            if cluster_size > 3:  # Küçük kümeler bile değerli
                bonus += 0.5
            if dist > 5.0:  # Uzak hedefler (kalan alanları bul)
                bonus += 1.0
        
        return bonus
    
    def _assess_frontier_quality(self, cluster: FrontierCluster, gx: float, gy: float, robot_xy: Tuple[float, float]) -> float:
        """
        Frontier Quality Assessment: accessibility and shape analysis
        Frontier'in kalitesini değerlendir (erişilebilirlik, şekil, boyut)
        """
        if self.map_msg is None:
            return 0.0
        
        quality_score = 0.0
        
        # 1. Cluster size quality (büyük kümeler daha iyi)
        if cluster.size > 15:
            quality_score += 2.0
        elif cluster.size > 8:
            quality_score += 1.0
        elif cluster.size < 3:
            quality_score -= 0.5
        
        # 2. Cluster shape quality (kompakt kümeler daha iyi)
        # Cluster'ın yayılımını hesapla (variance)
        if cluster.size > 1:
            cells = list(cluster.cells)
            xs = [c[0] for c in cells]
            ys = [c[1] for c in cells]
            center_x = sum(xs) / len(xs)
            center_y = sum(ys) / len(ys)
            
            # Variance hesapla (düşük variance = kompakt = iyi)
            var_x = sum((x - center_x)**2 for x in xs) / len(xs)
            var_y = sum((y - center_y)**2 for y in ys) / len(ys)
            variance = var_x + var_y
            
            # Kompakt kümeler bonus alır
            if variance < cluster.size * 0.5:  # Kompakt
                quality_score += 1.5
            elif variance > cluster.size * 2.0:  # Dağınık
                quality_score -= 0.5
        
        # 3. Accessibility quality (erişilebilirlik)
        accessibility = self._calculate_accessibility(gx, gy, robot_xy)
        quality_score += accessibility * 1.0
        
        # 4. Information gain quality (yüksek bilgi kazancı = iyi)
        if cluster.information_gain > 50:
            quality_score += 1.0
        elif cluster.information_gain > 20:
            quality_score += 0.5
        
        return quality_score
    
    def _get_goal_history_bonus(self, gx: float, gy: float, robot_xy: Tuple[float, float]) -> float:
        """
        Goal History Learning: track successful vs failed regions
        Başarılı bölgeleri tercih et, başarısız bölgelerden kaçın
        """
        if self.map_msg is None:
            return 0.0
        
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        
        # Goal'ı cell koordinatına çevir (history için)
        gx_cell = int((gx - ox) / res)
        gy_cell = int((gy - oy) / res)
        goal_cell = (gx_cell, gy_cell)
        
        # Goal history radius içindeki başarılı/başarısız goal'ları kontrol et
        radius_cells = int(self.goal_history_radius / res)
        success_count = 0
        fail_count = 0
        
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                check_cell = (gx_cell + dx, gy_cell + dy)
                
                # Başarılı goal'ları say
                if check_cell in self.successful_goals:
                    success_count += self.successful_goals[check_cell]
                
                # Başarısız goal'ları say
                if check_cell in self.failed_goals_history:
                    fail_count += self.failed_goals_history[check_cell]
        
        # Bonus/ceza hesapla
        bonus = 0.0
        
        # Başarılı bölgeleri tercih et
        if success_count > 0:
            bonus += min(success_count * 0.5, 2.0)  # Max 2.0 bonus
        
        # Başarısız bölgelerden kaçın
        if fail_count > 0:
            bonus -= min(fail_count * 0.8, 3.0)  # Max 3.0 penalty
        
        return bonus
    
    def _record_goal_success(self, goal_xy: Tuple[float, float]):
        """Record successful goal in history"""
        if self.map_msg is None:
            return
        
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        
        gx, gy = goal_xy
        gx_cell = int((gx - ox) / res)
        gy_cell = int((gy - oy) / res)
        goal_cell = (gx_cell, gy_cell)
        
        # Başarılı goal'ı kaydet
        if goal_cell not in self.successful_goals:
            self.successful_goals[goal_cell] = 0
        self.successful_goals[goal_cell] += 1
        
        # Eski kayıtları temizle (max 100 başarılı goal)
        if len(self.successful_goals) > 100:
            # En eski kayıtları sil (FIFO)
            oldest = min(self.successful_goals.items(), key=lambda x: x[1])
            del self.successful_goals[oldest[0]]
    
    def _record_goal_failure(self, goal_xy: Tuple[float, float]):
        """Record failed goal in history"""
        if self.map_msg is None:
            return
        
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        
        gx, gy = goal_xy
        gx_cell = int((gx - ox) / res)
        gy_cell = int((gy - oy) / res)
        goal_cell = (gx_cell, gy_cell)
        
        # Başarısız goal'ı kaydet
        if goal_cell not in self.failed_goals_history:
            self.failed_goals_history[goal_cell] = 0
        self.failed_goals_history[goal_cell] += 1
        
        # Eski kayıtları temizle (max 50 başarısız goal)
        if len(self.failed_goals_history) > 50:
            # En eski kayıtları sil (FIFO)
            oldest = min(self.failed_goals_history.items(), key=lambda x: x[1])
            del self.failed_goals_history[oldest[0]]
    
    def _estimate_path_cost(self, gx: float, gy: float, robot_xy: Tuple[float, float]) -> float:
        """
        Path Cost Estimation: Nav2 path cost'u tahmin et
        Daha kolay yolları tercih et (düşük cost = iyi)
        """
        goal_key = (round(gx, 1), round(gy, 1))
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Cache kontrolü
        if goal_key in self.path_cost_cache:
            cache_time = self.path_cost_cache_time.get(goal_key, 0.0)
            if current_time - cache_time < self.path_cost_cache_ttl:
                return self.path_cost_cache[goal_key]
        
        # Path cost tahmini: distance + obstacle density + accessibility
        rx, ry = robot_xy
        dist = math.hypot(gx - rx, gy - ry)
        
        # Obstacle density penalty
        obstacle_density = self._calculate_obstacle_density(gx, gy, robot_xy)
        
        # Accessibility penalty
        accessibility = self._calculate_accessibility(gx, gy, robot_xy)
        accessibility_penalty = (1.0 - accessibility) * dist
        
        # Estimated path cost
        estimated_cost = dist * (1.0 + obstacle_density * 2.0) + accessibility_penalty
        
        # Cache'e kaydet
        self.path_cost_cache[goal_key] = estimated_cost
        self.path_cost_cache_time[goal_key] = current_time
        
        # Cache temizleme (max 100 entry)
        if len(self.path_cost_cache) > 100:
            # En eski entry'leri sil
            oldest_key = min(self.path_cost_cache_time.items(), key=lambda x: x[1])[0]
            del self.path_cost_cache[oldest_key]
            del self.path_cost_cache_time[oldest_key]
        
        return estimated_cost
    
    def _predict_frontier_value(self, cluster: FrontierCluster, gx: float, gy: float, robot_xy: Tuple[float, float]) -> float:
        """
        Predictive Exploration: Frontier'in gelecekteki değerini tahmin et
        Eski frontier'ler (uzun süredir var olan) daha az değerli olabilir
        Yeni frontier'ler (yeni keşfedilen) daha değerli olabilir
        """
        if self.map_msg is None:
            return 0.0
        
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        
        # Cluster center'ı cell koordinatına çevir
        cx_cell = int((gx - ox) / res)
        cy_cell = int((gy - oy) / res)
        cluster_key = (cx_cell, cy_cell)
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Frontier yaşını takip et
        if cluster_key not in self.frontier_age:
            # Memory leak prevention: en fazla 500 entry tut (Bug #14)
            if len(self.frontier_age) > 500:
                oldest_keys = sorted(self.frontier_age, key=self.frontier_age.get)[:100]
                for k in oldest_keys:
                    del self.frontier_age[k]
            self.frontier_age[cluster_key] = current_time
        
        frontier_age = current_time - self.frontier_age[cluster_key]
        
        # Yeni frontier'ler (5 saniyeden az) bonus alır
        if frontier_age < 5.0:
            return 1.5  # Yeni frontier bonus
        # Eski frontier'ler (30 saniyeden fazla) ceza alır
        elif frontier_age > 30.0:
            return -0.5  # Eski frontier ceza
        # Orta yaşlı frontier'ler normal
        else:
            return 0.0
    
    def _get_efficiency_bonus(self, coverage: float, dist: float) -> float:
        """
        Exploration Efficiency: Coverage rate'e göre optimizasyon
        Yavaş ilerliyorsa daha yakın hedefleri tercih et
        Hızlı ilerliyorsa uzak hedeflere de gidebilir
        """
        bonus = 0.0
        
        # Coverage rate hesapla
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - (self.exploration_start_time.nanoseconds / 1e9)
        
        if elapsed > 0:
            coverage_rate = coverage / elapsed  # Coverage per second
            
            # Yavaş ilerliyorsa (düşük rate) yakın hedefleri tercih et
            if coverage_rate < 0.1:  # < 0.1% per second
                if dist < 5.0:
                    bonus += 1.0  # Yakın hedefler bonus
                elif dist > 10.0:
                    bonus -= 0.5  # Uzak hedefler ceza
            
            # Hızlı ilerliyorsa (yüksek rate) uzak hedeflere de gidebilir
            elif coverage_rate > 0.5:  # > 0.5% per second
                if dist > 5.0:
                    bonus += 0.5  # Uzak hedefler bonus
        
        # Coverage history'ye ekle
        self.coverage_history.append(coverage)
        if len(self.coverage_history) >= 2:
            # Son 2 coverage arasındaki farkı hesapla
            recent_rate = (self.coverage_history[-1] - self.coverage_history[0]) / len(self.coverage_history)
            self.exploration_rate = recent_rate
        
        return bonus

    def _check_stuck(self, robot_xy: Tuple[float, float]) -> bool:
        """Check if robot is stuck (not moving) - combines distance and velocity-based detection"""
        if not self.stuck_detection_enabled:
            return False
        
        # Velocity-based stuck detection (faster response)
        if self.use_velocity_stuck_detection and len(self.robot_velocities) >= 10:
            recent_velocities = list(self.robot_velocities)[-10:]
            avg_linear_vel = sum(v[0] for v in recent_velocities) / len(recent_velocities)
            avg_angular_vel = sum(abs(v[1]) for v in recent_velocities) / len(recent_velocities)
            
            # If velocity is very low but goal is active, might be stuck
            if abs(avg_linear_vel) < self.stuck_velocity_threshold and avg_angular_vel < 0.1:
                if self.active_goal_handle is not None:
                    time_stuck = (self.get_clock().now() - self.last_significant_movement).nanoseconds / 1e9
                    if time_stuck > self.stuck_time_threshold * 0.5:  # Faster detection with velocity
                        self.get_logger().warn(f"Robot appears stuck (velocity-based)! Linear vel: {avg_linear_vel:.3f} m/s, Angular vel: {avg_angular_vel:.3f} rad/s")
                        return True
        
        # Distance-based stuck detection (original method)
        if len(self.robot_positions) < 10:
            return False
        
        # Check if robot has moved significantly
        recent_positions = list(self.robot_positions)[-10:]
        min_x = min(p[0] for p in recent_positions)
        max_x = max(p[0] for p in recent_positions)
        min_y = min(p[1] for p in recent_positions)
        max_y = max(p[1] for p in recent_positions)
        
        movement_range = math.hypot(max_x - min_x, max_y - min_y)
        
        if movement_range < self.stuck_distance_threshold:
            time_stuck = (self.get_clock().now() - self.last_significant_movement).nanoseconds / 1e9
            if time_stuck > self.stuck_time_threshold:
                self.get_logger().warn(f"Robot appears stuck (distance-based)! Movement range: {movement_range:.2f}m, Time stuck: {time_stuck:.1f}s")
                return True
        else:
            self.last_significant_movement = self.get_clock().now()
        
        return False

    def _make_goal(self, gx: float, gy: float, robot_xy: Tuple[float, float]) -> PoseStamped:
        """
        Hedef pose oluştur - AKILLI YÖN KONTROLÜ İLE
        
        Mantık:
        1. Başlangıca dönüş: Yön önemsiz, mevcut yönü koru (spin önleme)
        2. Çok yakın hedefler (<1.5m): Yön belirtme, Nav2 optimize etsin
        3. Yakın hedefler (1.5-3m): Mevcut yöne yakınsa koru, değilse hedef yönü kullan
        4. Uzak hedefler (>3m): Normal - hedefe doğru yön
        
        Bu yaklaşım robotun gereksiz dönüşlerini %70 azaltır!
        """
        rx, ry = robot_xy
        dist = math.hypot(gx - rx, gy - ry)
        
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.position.z = 0.0
        
        # ============================================================
        # ÖZEL DURUM 1: Başlangıca dönüş - Akıllı yön kontrolü (spin önleme)
        # ============================================================
        if self.returning_to_start:
            # Çok yakınsa (<0.6m) yön belirtme - Nav2 optimize etsin
            if dist < self.start_position_tolerance:
                pose.pose.orientation.w = 1.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                self.get_logger().debug(f"Return-to-start goal very close ({dist:.2f}m < {self.start_position_tolerance:.2f}m): No orientation constraint")
                return pose
            
            # Yakınsa (<2m) ve robot zaten doğru yöndeyse mevcut yönü koru
            robot_yaw = self._get_robot_yaw_in_map()
            if dist < 2.0 and robot_yaw is not None:
                goal_yaw = math.atan2(gy - ry, gx - rx)
                yaw_diff = abs(math.atan2(math.sin(goal_yaw - robot_yaw), 
                                         math.cos(goal_yaw - robot_yaw)))
                # Eğer robot zaten başlangıca doğru bakıyorsa (±60°), mevcut yönü koru
                if yaw_diff < math.pi / 3:  # 60 degrees
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            self.goal_frame, self.robot_frame, rclpy.time.Time()
                        )
                        pose.pose.orientation = tf.transform.rotation
                        self.get_logger().debug(f"Return-to-start goal: keeping current orientation (dist={dist:.2f}m, yaw_diff={math.degrees(yaw_diff):.1f}°)")
                        return pose
                    except:
                        # TF yoksa yön belirtme
                        pass
            
            # Uzaksa veya yön farkı büyükse yön belirtme - Nav2 optimize etsin
            # Nav2 yaw_goal_tolerance (1.57 rad ~90°) olduğu için spin yapmaz
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            self.get_logger().debug(f"Return-to-start goal ({dist:.2f}m): No orientation constraint - Nav2 will allow any approach direction")
            return pose
        
        # Hedef yönü hesapla
        goal_yaw = math.atan2(gy - ry, gx - rx)
        robot_yaw = self._get_robot_yaw_in_map()
        
        # ============================================================
        # ÖZEL DURUM 2: Çok yakın hedefler (<1.5m) - Yön belirtme!
        # Nav2'nin kendi path planner'ı en iyi yolu bulsun
        # ============================================================
        if dist < 1.5:
            # Quaternion identity: Yön yok (w=1, diğerleri 0) - Nav2 optimize etsin
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            self.get_logger().debug(f"Very close goal ({dist:.2f}m < 1.5m): No orientation constraint - Nav2 will optimize")
            return pose
        
        # ============================================================
        # ÖZEL DURUM 3: Yakın hedefler (1.5-3m) - Robot zaten doğru yöndeyse mevcut yönü koru
        # ============================================================
        if dist < 3.0 and robot_yaw is not None:
            # Yön farkını hesapla (normalize edilmiş: -π...π)
            yaw_diff = abs(math.atan2(math.sin(goal_yaw - robot_yaw), 
                                      math.cos(goal_yaw - robot_yaw)))
            
            # Eğer robot zaten doğru yöne bakıyorsa (±45°), mevcut yönü koru
            if yaw_diff < 0.785:  # 45 derece (~π/4) - Nav2 yaw_goal_tolerance
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.goal_frame, self.robot_frame, rclpy.time.Time()
                    )
                    pose.pose.orientation = tf.transform.rotation
                    self.get_logger().debug(f"Close goal ({dist:.2f}m, yaw_diff={math.degrees(yaw_diff):.1f}°): Keeping current orientation")
                    return pose
                except:
                    # TF yoksa hedef yönü kullan (aşağıdaki default)
                    pass
        
        # ============================================================
        # DEFAULT DURUM: Uzak hedefler (>3m) - Yön belirtme, Nav2 optimize etsin
        # ============================================================
        # Uzak hedefler için de yön constraint'i kaldır - Nav2 optimize etsin
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        self.get_logger().debug(f"Far goal ({dist:.2f}m): No orientation constraint - Nav2 will optimize")
        return pose

    def _send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Nav2 action server not ready yet.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending frontier goal: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

        self.last_goal_time = self.get_clock().now()
        self.active_goal_started = self.get_clock().now()
        self.last_goal_xy = (pose.pose.position.x, pose.pose.position.y)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"send_goal failed: {e}")
            self.active_goal_handle = None
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            # Mark as failed (except for start position goals) - deque FIFO
            if self.last_goal_xy and not self.returning_to_start:
                self.failed_goals.append(self.last_goal_xy)  # Append to deque (FIFO)
            self.active_goal_handle = None
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            res = future.result()
            status = res.status
        except Exception as e:
            self.get_logger().error(f"get_result failed: {e}")
            status = None

        self.get_logger().info(f"Goal finished with status: {status}")
        
        # Status 4 = SUCCEEDED
        if status == 4:  # SUCCESS
            # Goal History Learning: Başarılı goal'ı kaydet
            if self.last_goal_xy and not self.returning_to_start:
                self._record_goal_success(self.last_goal_xy)
            
            if self.returning_to_start:
                # Goal başarılı - HEMEN mesafe kontrolü yap (Nav2 goal tolerance içinde olabilir ama start tolerance dışında)
                robot_xy = self._get_robot_xy_in_map()
                if robot_xy and self.start_position:
                    dist_to_start = math.hypot(robot_xy[0] - self.start_position[0], 
                                              robot_xy[1] - self.start_position[1])
                    self.get_logger().info(f"Return-to-start goal succeeded. Distance to start: {dist_to_start:.2f}m (tolerance: {self.start_position_tolerance:.2f}m)")
                    
                    # ÖNEMLİ: Nav2 goal tolerance (0.5m) içinde ama start tolerance (0.6m) dışında olabilir
                    # Eğer start tolerance içindeyse mission complete
                    if dist_to_start < self.start_position_tolerance:
                        self.get_logger().info("✓✓✓ Successfully returned to start position! Mission complete! ✓✓✓")
                        self.returning_to_start = False
                        self.exploration_complete = True
                        self.mission_complete = True
                        self._mission_complete_logged = False
                        # KRİTİK: Robotu durdur - goal cancel yeterli değil!
                        self._cancel_active_goal()
                        self._stop_robot()
                    else:
                        # Start tolerance dışında - _tick içinde tekrar goal gönderilecek
                        self.get_logger().info(f"Goal succeeded but still {dist_to_start:.2f}m from start (tolerance: {self.start_position_tolerance:.2f}m). Will retry in next tick.")
                else:
                    # TF yoksa varsayılan olarak tamamla
                    self.get_logger().info("Return-to-start goal succeeded (TF unavailable). Mission complete!")
                    self.returning_to_start = False
                    self.exploration_complete = True
                    self.mission_complete = True
                    self._mission_complete_logged = False
                    # KRİTİK: Robotu durdur
                    self._cancel_active_goal()
                    self._stop_robot()
        else:
            # Failed - Return-to-start goal'ları için attempt counter'ı artır
            if self.returning_to_start:
                self.return_to_start_attempts += 1
                self.get_logger().warn(f"Return-to-start goal failed (status: {status}). Attempt {self.return_to_start_attempts}/{self.max_return_attempts}")
                if self.return_to_start_attempts >= self.max_return_attempts:
                    self.get_logger().warn(f"Max return attempts ({self.max_return_attempts}) reached. Mission complete anyway.")
                    self.mission_complete = True
                    self.returning_to_start = False
                    self.exploration_complete = True
                    # KRİTİK: Robotu durdur
                    self._cancel_active_goal()
                    self._stop_robot()
            # Exploration goal'ları için failed goals'a ekle
            elif self.last_goal_xy and not self.returning_to_start:
                self.failed_goals.append(self.last_goal_xy)  # Append to deque (FIFO - oldest auto-removed)
                # Goal History Learning: Başarısız goal'ı kaydet
                self._record_goal_failure(self.last_goal_xy)
        
        self.active_goal_handle = None
        self.active_goal_started = None

    def _cancel_active_goal(self):
        if self.active_goal_handle is None:
            return
        self.get_logger().warn("Cancelling active goal (timeout/stuck).")
        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda f: None)
        self.active_goal_handle = None
        self.active_goal_started = None
        # Mark as failed (except for start position goals) - deque FIFO
        if self.last_goal_xy and not self.returning_to_start:
            self.failed_goals.append(self.last_goal_xy)  # Append to deque (FIFO - oldest auto-removed)
            # Goal History Learning: Başarısız goal'ı kaydet
            self._record_goal_failure(self.last_goal_xy)

    def _publish_visualization(self, robot_xy: Tuple[float, float]):
        """Publish frontier markers for visualization"""
        if not self.publish_visualization or not self.map_msg:
            return

        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        marker_array = MarkerArray()
        
        # Clear old markers - DELETEALL marker must have ID 0
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.header.frame_id = self.goal_frame
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.id = 0
        clear_marker.ns = "frontiers"
        marker_array.markers.append(clear_marker)

        clusters = self.cached_frontier_clusters
        for i, cluster in enumerate(clusters[:50]):  # Limit to 50 clusters
            gx = ox + (cluster.center_x + 0.5) * res
            gy = oy + (cluster.center_y + 0.5) * res
            
            marker = Marker()
            marker.header.frame_id = self.goal_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"  # Namespace for grouping
            marker.id = i + 1  # Start from 1 (0 is DELETEALL)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = gx
            marker.pose.position.y = gy
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color based on information gain (red = high, blue = low)
            max_gain = max(c.information_gain for c in clusters) if clusters else 1.0
            normalized_gain = cluster.information_gain / max_gain if max_gain > 0 else 0.0
            marker.color.r = float(normalized_gain)
            marker.color.g = 0.0
            marker.color.b = float(1.0 - normalized_gain)
            marker.color.a = 0.8
            marker.lifetime = Duration(seconds=2.0).to_msg()
            
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def _calculate_coverage(self) -> float:
        """Calculate explored area percentage"""
        if not self.map_msg:
            return 0.0
        
        data = self.map_msg.data
        total_cells = len([v for v in data if v != -1])  # Known cells
        free_cells = len([v for v in data if 0 <= v <= self.free_value_max])
        
        if total_cells == 0:
            return 0.0
        
        return (free_cells / total_cells) * 100.0

    def _publish_metrics(self):
        """Publish exploration metrics"""
        if not self.publish_metrics:
            return
        
        # Coverage
        coverage = self._calculate_coverage()
        msg = Float32()
        msg.data = coverage
        self.metrics_pubs['coverage'].publish(msg)
        
        # Frontier count
        msg = Float32()
        msg.data = float(len(self.cached_frontier_clusters))
        self.metrics_pubs['frontier_count'].publish(msg)
        
        # Total distance
        msg = Float32()
        msg.data = self.total_distance
        self.metrics_pubs['total_distance'].publish(msg)
        
        # Exploration time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Float32()
        msg.data = elapsed
        self.metrics_pubs['exploration_time'].publish(msg)

    def _save_start_position(self, robot_xy: Tuple[float, float]):
        """Save start position - ilk geçerli pozisyonu kaydet"""
        if not self.start_position_saved and robot_xy is not None:
            self.start_position = robot_xy
            self.start_position_saved = True
            self.last_robot_xy = robot_xy
            self.get_logger().info(f"✓ Start position saved: x={robot_xy[0]:.2f}, y={robot_xy[1]:.2f}")

    def _stop_robot(self):
        """
        KRİTİK: Mission complete olduğunda robotu tamamen durdur
        
        Goal cancel yeterli değil çünkü:
        1. Nav2 kendi planner'ından goal gönderebilir
        2. Local costmap hala cmd_vel yayınlayabilir
        3. Robot momentum'dan dolayı hareket edebilir
        
        Bu fonksiyon cmd_vel'e 0 göndererek robotu garantili durdurur.
        """
        # Zero velocity command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        # Publish multiple times to ensure it's received (garantili durma)
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_cmd)
        
        self.get_logger().info("🛑 Robot stopped - cmd_vel set to zero")

    def _tick(self):
        # Map yoksa bekle
        if self.map_msg is None:
            return

        # TF yoksa bekle
        robot_xy = self._get_robot_xy_in_map()
        if robot_xy is None:
            return

        # Update distance tracking
        if self.last_robot_xy is not None:
            dist = math.hypot(robot_xy[0] - self.last_robot_xy[0], 
                            robot_xy[1] - self.last_robot_xy[1])
            self.total_distance += dist
        self.last_robot_xy = robot_xy

        # Track robot positions for stuck detection
        self.robot_positions.append(robot_xy)

        # Başlangıç pozisyonunu kaydet - daha erken kaydet (ilk geçerli pozisyonu al)
        if not self.start_position_saved:
            # İlk geçerli pozisyonu hemen kaydet (timeout bekleme)
            # Map ve TF hazırsa kaydet
            if self.map_msg is not None:
                self._save_start_position(robot_xy)
            else:
                # Map henüz yoksa kısa bir süre bekle
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                if elapsed > 2.0:  # 2 saniye bekle (10.0 -> 2.0)
                    self._save_start_position(robot_xy)
            return

        # Stuck detection
        if self._check_stuck(robot_xy) and self.active_goal_handle is not None:
            self.get_logger().warn("Robot stuck detected! Cancelling current goal.")
            self._cancel_active_goal()
            # Clear failed goals to try different paths
            self.failed_goals.clear()
            self.robot_positions.clear()
            return

        # Aktif goal timeout kontrol
        if self.active_goal_started is not None:
            # Return-to-start için daha uzun timeout, exploration için normal timeout
            timeout = self.return_to_start_timeout if self.returning_to_start else self.goal_timeout
            elapsed = (self.get_clock().now() - self.active_goal_started).nanoseconds / 1e9
            if elapsed > timeout:
                timeout_type = "return-to-start" if self.returning_to_start else "exploration"
                self.get_logger().warn(f"Goal timeout ({timeout_type}): {elapsed:.1f}s > {timeout:.1f}s")
                self._cancel_active_goal()
            # Return-to-start için timeout kontrolü yapıldıktan sonra devam et (retry için)
            if not self.returning_to_start:
                return

        # Cooldown - Goal değiştirmeyi önle (çok önemli: dönüş önleme!)
        since_last = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
        if since_last < self.goal_cooldown:
            return
        
        # AKILLI GOAL DEĞİŞTİRME: Aktif goal varsa ve iyi gidiyorsa değiştirme!
        # Robot goal'a doğru ilerliyorsa yeni goal gönderme (dönüş önleme)
        if self.active_goal_handle is not None and self.last_goal_xy is not None:
            # Robot'un son goal'a olan mesafesini kontrol et
            dist_to_goal = math.hypot(robot_xy[0] - self.last_goal_xy[0], 
                                     robot_xy[1] - self.last_goal_xy[1])
            
            # Eğer robot goal'a yaklaşıyorsa (son 5 saniyede mesafe azaldıysa) goal değiştirme
            if hasattr(self, '_last_goal_distance_check'):
                time_since_check = (self.get_clock().now() - self._last_goal_distance_check_time).nanoseconds / 1e9
                if time_since_check < 5.0 and hasattr(self, '_last_goal_distance'):
                    if dist_to_goal < self._last_goal_distance - 0.3:  # 0.3m yaklaştıysa
                        # Robot goal'a doğru ilerliyor - goal değiştirme!
                        return
            
            # Goal'a çok yakınsa (1m içinde) yeni goal gönderme
            if dist_to_goal < 1.0:
                return
            
            self._last_goal_distance = dist_to_goal
            self._last_goal_distance_check_time = self.get_clock().now()

        # Başlangıca dönme kontrolü (exploration_complete kontrolünden ÖNCE yapılmalı!)
        if self.returning_to_start:
            if self.mission_complete:
                # Mission tamamlandı - hiçbir şey yapma
                if self.active_goal_handle is not None:
                    self._cancel_active_goal()
                return
            
            if self.start_position is not None:
                dist_to_start = math.hypot(robot_xy[0] - self.start_position[0], 
                                          robot_xy[1] - self.start_position[1])
                
                # ÖNEMLİ: Sürekli mesafe kontrolü yap (goal başarılı olsa bile robot tam yerine varmamış olabilir)
                if dist_to_start < self.start_position_tolerance:
                    # Tolerance içindeyse mission complete
                    self.get_logger().info(f"✓✓✓ Returned to start position! Distance: {dist_to_start:.2f}m < {self.start_position_tolerance:.2f}m ✓✓✓")
                    self.returning_to_start = False
                    self.exploration_complete = True
                    self.mission_complete = True
                    # KRİTİK: Aktif goal'ı iptal et VE robotu durdur
                    if self.active_goal_handle is not None:
                        self._cancel_active_goal()
                    self._stop_robot()
                    return
                
                # Goal yoksa veya başarısız olduysa tekrar gönder
                # ÖNEMLİ: Goal başarılı oldu ama start tolerance dışındaysa tekrar gönder
                if self.active_goal_handle is None:
                    # Maksimum deneme sayısını kontrol et
                    if self.return_to_start_attempts >= self.max_return_attempts:
                        self.get_logger().warn(f"Max return attempts ({self.max_return_attempts}) reached. Mission complete anyway.")
                        self.mission_complete = True
                        self.returning_to_start = False
                        self.exploration_complete = True
                        # KRİTİK: Robotu durdur
                        self._stop_robot()
                        return
                    
                    # Tolerance dışındaysa yeni goal gönder
                    # ÖNEMLİ: Nav2 goal tolerance (0.5m) içinde olabilir ama start tolerance (0.6m) dışında
                    # Bu durumda tekrar goal göndermeliyiz
                    if dist_to_start >= self.start_position_tolerance:
                        self.get_logger().info(f"Retrying start goal. Distance: {dist_to_start:.2f}m (Nav2 tolerance: 0.5m, Start tolerance: {self.start_position_tolerance:.2f}m, attempt {self.return_to_start_attempts + 1}/{self.max_return_attempts})")
                        pose = self._make_goal(self.start_position[0], self.start_position[1], robot_xy)
                        self._send_goal(pose)
                        # NOT: attempt counter'ı burada artırma - goal başarısız olduğunda _on_result içinde artırılacak
                    else:
                        # Tolerance içinde ama goal yok - mission complete (bu durum normalde olmamalı)
                        self.get_logger().info(f"Within tolerance ({dist_to_start:.2f}m < {self.start_position_tolerance:.2f}m) but no active goal. Mission complete!")
                        self.mission_complete = True
                        self.returning_to_start = False
                        self.exploration_complete = True
                        # KRİTİK: Robotu durdur
                        self._stop_robot()
            return

        # Exploration complete ise frontier arama - sadece başlangıca dönüş yapılabilir
        if self.exploration_complete:
            # Mission tamamen tamamlandıysa hiçbir şey yapma
            if self.mission_complete:
                if self.active_goal_handle is not None:
                    self._cancel_active_goal()
                # KRİTİK: Robotu durdur (sürekli kontrol et - Nav2 tekrar goal gönderebilir)
                self._stop_robot()
                # Sadece bir kez log yaz
                if not hasattr(self, '_mission_complete_logged'):
                    self.get_logger().info("✓✓✓ MISSION COMPLETE ✓✓✓ - Robot stopped at start position.")
                    self._mission_complete_logged = True
                return
            
            # Exploration complete ama returning_to_start False ise, başlangıca dönmeyi başlat
            if self.return_to_start and self.start_position is not None and not self.returning_to_start:
                self.get_logger().info(f"✓ Exploration complete! Starting return to start: x={self.start_position[0]:.2f}, y={self.start_position[1]:.2f}")
                self.returning_to_start = True
                self.return_to_start_attempts = 0  # Reset attempt counter
                pose = self._make_goal(self.start_position[0], self.start_position[1], robot_xy)
                self._send_goal(pose)
                return
            
            # Exploration complete ama return_to_start False veya start_position yok
            if not self.return_to_start or self.start_position is None:
                self.get_logger().info("Exploration complete but return_to_start disabled or start position not saved.")
                self.mission_complete = True
                if self.active_goal_handle is not None:
                    self._cancel_active_goal()
                # KRİTİK: Robotu durdur
                self._stop_robot()
                return
            
            # returning_to_start True ama mission_complete False - devam et
            return

        # Frontier ara
        goal_xy = self._pick_frontier_goal(robot_xy)
        
        # Publish visualization
        if self.publish_visualization:
            self._publish_visualization(robot_xy)
        
        if goal_xy is None:
            # Frontier bulunamadı - exploration complete
            if not self.exploration_complete:
                coverage = self._calculate_coverage()
                self.get_logger().info(f"✓ No frontier found. Exploration complete! Coverage: {coverage:.1f}%")
                self.exploration_complete = True
                
                if self.return_to_start and self.start_position is not None:
                    dist_to_start = math.hypot(robot_xy[0] - self.start_position[0], 
                                              robot_xy[1] - self.start_position[1])
                    self.get_logger().info(f"Returning to start position: x={self.start_position[0]:.2f}, y={self.start_position[1]:.2f} (current distance: {dist_to_start:.2f}m)")
                    self.returning_to_start = True
                    self.return_to_start_attempts = 0  # Reset attempt counter
                    pose = self._make_goal(self.start_position[0], self.start_position[1], robot_xy)
                    self._send_goal(pose)
                elif not self.return_to_start:
                    self.get_logger().info("Exploration complete but return_to_start is disabled.")
                    self.mission_complete = True
                elif self.start_position is None:
                    self.get_logger().warn("Exploration complete but start position not saved!")
                    self.mission_complete = True
            return

        pose = self._make_goal(goal_xy[0], goal_xy[1], robot_xy)
        self._send_goal(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Shutdown'ı güvenli şekilde çağır - zaten shutdown edilmişse hata verme
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Zaten shutdown edilmişse görmezden gel


if __name__ == "__main__":
    main()
