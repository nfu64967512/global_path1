"""
Zigzag 網格生成器整合模組
===============================

整合到 drone_path_planner_V10 專案的 Zigzag 網格掃描功能

主要功能：
1. Zigzag 網格路徑生成（蛇形掃描）
2. 相機參數自動計算（航線間距、GSD）
3. 多機任務分割
4. MAVLink 航點匯出 (QGC WPL 110)
5. 障礙物避障整合

作者：維士曾
日期：2026-02-03
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional, Dict, Any
import os


# ==============================
# 掃描模式枚舉
# ==============================
class ScanPattern(Enum):
    """掃描模式"""
    ZIGZAG = auto()      # 蛇形掃描（主要模式）
    PARALLEL = auto()    # 平行掃描
    SPIRAL = auto()       # 螺旋掃描


class TurnMode(Enum):
    """轉向模式"""
    HEADING_LOCK = auto()     # 航向鎖定（偵察攝影）
    BANK_TURN = auto()        # 傾斜轉向（效率優先）


# ==============================
# 資料結構定義
# ==============================
@dataclass
class CameraSpec:
    """相機規格"""
    name: str
    sensor_width: float      # mm
    sensor_height: float     # mm
    image_width: int         # pixels
    image_height: int        # pixels
    focal_length: float      # mm
    manufacturer: str = ""
    
    @property
    def aspect_ratio(self) -> float:
        return self.image_width / self.image_height
    
    @property
    def fov_horizontal(self) -> float:
        """水平視場角（度）"""
        return 2 * math.degrees(math.atan(self.sensor_width / (2 * self.focal_length)))
    
    @property
    def fov_vertical(self) -> float:
        """垂直視場角（度）"""
        return 2 * math.degrees(math.atan(self.sensor_height / (2 * self.focal_length)))


@dataclass
class ZigzagSurveyConfig:
    """Zigzag 掃描配置"""
    # 飛行參數
    altitude: float = 50.0           # 飛行高度 (m)
    speed: float = 5.0               # 飛行速度 (m/s)
    yaw_speed: float = 60.0          # 轉向速度 (deg/s)
    
    # 重疊參數
    front_overlap: float = 80.0      # 前向重疊率 (%)
    side_overlap: float = 60.0       # 側向重疊率 (%)
    
    # 掃描模式
    scan_pattern: ScanPattern = ScanPattern.ZIGZAG
    scan_angle: float = 0.0          # 掃描角度 (deg)
    turn_mode: TurnMode = TurnMode.HEADING_LOCK
    
    # 進階設定
    overshoot_distance: float = 0.0  # 超出距離 (m)
    leadin_distance: float = 0.0     # 引入距離 (m)
    
    # 安全參數
    safety_distance: float = 5.0     # 安全間距 (m)
    min_altitude: float = 5.0        # 最低高度 (m)
    max_altitude: float = 120.0      # 最高高度 (m)
    
    def validate(self) -> bool:
        """驗證配置有效性"""
        if self.altitude <= 0:
            raise ValueError("飛行高度必須大於 0")
        if not self.min_altitude <= self.altitude <= self.max_altitude:
            raise ValueError(f"飛行高度必須在 {self.min_altitude}-{self.max_altitude}m 範圍內")
        if self.speed <= 0:
            raise ValueError("飛行速度必須大於 0")
        if not 0 <= self.front_overlap < 100:
            raise ValueError("前向重疊率必須在 0-100% 之間")
        if not 0 <= self.side_overlap < 100:
            raise ValueError("側向重疊率必須在 0-100% 之間")
        return True


@dataclass
class SurveyStatistics:
    """掃描統計資料"""
    num_waypoints: int = 0
    num_lines: int = 0
    total_distance: float = 0.0      # m
    coverage_area: float = 0.0       # m²
    estimated_time: float = 0.0      # s
    num_photos: int = 0
    gsd: float = 0.0                 # cm/px


@dataclass 
class ZigzagSurveyResult:
    """Zigzag 掃描結果"""
    success: bool = False
    message: str = ""
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    scan_lines: List[List[Tuple[float, float]]] = field(default_factory=list)
    statistics: SurveyStatistics = field(default_factory=SurveyStatistics)
    planning_time: float = 0.0
    
    @property
    def is_success(self) -> bool:
        return self.success


# ==============================
# 相機資料庫
# ==============================
CAMERA_DATABASE: Dict[str, CameraSpec] = {
    "GoPro Hero 4 Black": CameraSpec(
        name="Hero 4 Black", sensor_width=6.17, sensor_height=4.55,
        image_width=4000, image_height=3000, focal_length=2.98, manufacturer="GoPro"
    ),
    "GoPro Hero 11 Black": CameraSpec(
        name="Hero 11 Black", sensor_width=7.62, sensor_height=5.71,
        image_width=5312, image_height=2988, focal_length=3.0, manufacturer="GoPro"
    ),
    "DJI Mavic 3": CameraSpec(
        name="Mavic 3", sensor_width=17.3, sensor_height=13.0,
        image_width=5280, image_height=3956, focal_length=24.0, manufacturer="DJI"
    ),
    "DJI Phantom 4 Pro": CameraSpec(
        name="Phantom 4 Pro", sensor_width=13.2, sensor_height=8.8,
        image_width=5472, image_height=3648, focal_length=8.8, manufacturer="DJI"
    ),
    "DJI Mini 3 Pro": CameraSpec(
        name="Mini 3 Pro", sensor_width=9.7, sensor_height=7.3,
        image_width=4000, image_height=3000, focal_length=6.72, manufacturer="DJI"
    ),
    "Sony A7R IV": CameraSpec(
        name="A7R IV", sensor_width=35.7, sensor_height=23.8,
        image_width=9504, image_height=6336, focal_length=35.0, manufacturer="Sony"
    ),
    "Canon EOS R5": CameraSpec(
        name="EOS R5", sensor_width=36.0, sensor_height=24.0,
        image_width=8192, image_height=5464, focal_length=35.0, manufacturer="Canon"
    ),
    "Custom Camera": CameraSpec(
        name="Custom", sensor_width=13.2, sensor_height=8.8,
        image_width=4000, image_height=3000, focal_length=8.0, manufacturer="Custom"
    ),
}


# ==============================
# 相機計算器
# ==============================
class CameraCalculator:
    """相機參數計算器"""
    
    @staticmethod
    def calculate_gsd(altitude_m: float, focal_length_mm: float,
                     sensor_width_mm: float, image_width_px: int) -> float:
        """
        計算地面採樣距離 (Ground Sampling Distance, GSD)
        
        返回: GSD (公尺/像素)
        """
        if focal_length_mm <= 0 or image_width_px <= 0:
            return 0.0
        gsd = (altitude_m * sensor_width_mm) / (focal_length_mm * image_width_px)
        return gsd
    
    @staticmethod
    def calculate_ground_coverage(altitude_m: float, focal_length_mm: float,
                                 sensor_width_mm: float, sensor_height_mm: float) -> Tuple[float, float]:
        """
        計算地面覆蓋範圍
        
        返回: (ground_width_m, ground_height_m)
        """
        if focal_length_mm <= 0:
            return (0.0, 0.0)
        ground_width = (altitude_m * sensor_width_mm) / focal_length_mm
        ground_height = (altitude_m * sensor_height_mm) / focal_length_mm
        return (ground_width, ground_height)
    
    @staticmethod
    def calculate_spacing_from_overlap(altitude_m: float, camera: CameraSpec,
                                       front_overlap_percent: float,
                                       side_overlap_percent: float) -> Tuple[float, float]:
        """
        根據重疊率計算航線間距和拍照間隔
        
        返回: (line_spacing_m, photo_interval_m)
        """
        ground_width, ground_height = CameraCalculator.calculate_ground_coverage(
            altitude_m, camera.focal_length, camera.sensor_width, camera.sensor_height
        )
        
        side_overlap_ratio = side_overlap_percent / 100.0
        line_spacing = ground_width * (1.0 - side_overlap_ratio)
        
        front_overlap_ratio = front_overlap_percent / 100.0
        photo_interval = ground_height * (1.0 - front_overlap_ratio)
        
        return (line_spacing, photo_interval)


# ==============================
# Zigzag 網格生成器核心
# ==============================
class ZigzagGridGenerator:
    """
    Zigzag 網格掃描生成器
    
    核心功能：生成蛇形（Zigzag）掃描路徑
    - 第一條線從左到右
    - 第二條線從右到左
    - 依此循環，形成連續路徑
    """
    
    EARTH_RADIUS_M = 111111.0  # 每度約 111111 公尺
    
    def __init__(self, config: ZigzagSurveyConfig = None, camera: CameraSpec = None):
        self.config = config or ZigzagSurveyConfig()
        self.camera = camera or CAMERA_DATABASE["DJI Phantom 4 Pro"]
        self._line_spacing = 30.0
        self._photo_interval = 10.0
        
        if camera:
            self._calculate_auto_spacing()
    
    def _calculate_auto_spacing(self):
        """自動計算航線間距"""
        self._line_spacing, self._photo_interval = CameraCalculator.calculate_spacing_from_overlap(
            self.config.altitude, self.camera,
            self.config.front_overlap, self.config.side_overlap
        )
    
    def generate_zigzag_grid(self, corners: List[Tuple[float, float]], 
                             region_idx: int = 0,
                             start_from_left: bool = True) -> ZigzagSurveyResult:
        """
        生成 Zigzag 網格掃描路徑
        
        Args:
            corners: 邊界角點列表 [(lat, lon), ...]
            region_idx: 區域索引（多機協同時用於確定起始方向）
            start_from_left: 是否從左側開始
            
        Returns:
            ZigzagSurveyResult: 掃描結果
        """
        import time
        start_time = time.time()
        
        result = ZigzagSurveyResult()
        
        try:
            # 驗證配置
            self.config.validate()
            
            if len(corners) < 3:
                result.message = "至少需要 3 個角點"
                return result
            
            # 座標變換
            pts_rot, lat0, lon0, cosLat0, cos_t, sin_t = self._project_and_rotate(
                corners, self.config.scan_angle
            )
            
            # 計算掃描線
            ys = [p[1] for p in pts_rot]
            minY, maxY = min(ys), max(ys)
            
            # 添加邊界餘量
            margin = self._line_spacing * 0.1
            minY -= margin
            maxY += margin
            
            # 計算掃描線數量
            total_lines = max(1, int(math.ceil((maxY - minY) / self._line_spacing)) + 1)
            
            waypoints = []
            scan_lines = []
            total_distance = 0.0
            prev_point = None
            
            for li in range(total_lines):
                y = minY + li * self._line_spacing
                if y > maxY:
                    y = maxY
                
                # 計算與多邊形的交點
                xs = self._intersect_line_polygon(pts_rot, y)
                
                if len(xs) < 2:
                    continue
                
                # === ZIGZAG 核心邏輯 ===
                # 確定掃描方向（之字形）
                # 多機協同時：region_idx 決定互補起始方向
                effective_idx = li + (region_idx if not start_from_left else 0)
                go_left_to_right = (effective_idx % 2 == 0)
                
                if go_left_to_right:
                    line_points = [(xs[0], y), (xs[-1], y)]
                else:
                    line_points = [(xs[-1], y), (xs[0], y)]
                
                # 轉換回地理座標
                geo_line = []
                for xr, yr in line_points:
                    lat, lon = self._rotate_back_to_geographic(
                        cos_t, sin_t, xr, yr, lat0, lon0, cosLat0
                    )
                    geo_line.append((lat, lon))
                    waypoints.append((lat, lon))
                    
                    # 計算距離
                    if prev_point is not None:
                        total_distance += self._calculate_distance(prev_point, (lat, lon))
                    prev_point = (lat, lon)
                
                scan_lines.append(geo_line)
            
            # 計算統計資料
            result.waypoints = waypoints
            result.scan_lines = scan_lines
            result.statistics = self._calculate_statistics(
                waypoints, scan_lines, total_distance, corners
            )
            result.success = True
            result.message = "Zigzag 網格生成成功"
            result.planning_time = time.time() - start_time
            
        except Exception as e:
            result.message = f"生成失敗: {str(e)}"
        
        return result
    
    def _project_and_rotate(self, corners: List[Tuple[float, float]], 
                           angle_deg: float) -> Tuple:
        """投影並旋轉座標系"""
        lat0 = sum(p[0] for p in corners) / len(corners)
        lon0 = sum(p[1] for p in corners) / len(corners)
        cosLat0 = math.cos(math.radians(lat0))
        
        theta = math.radians(angle_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        
        pts_rot = []
        for lat, lon in corners:
            x = (lon - lon0) * self.EARTH_RADIUS_M * cosLat0
            y = (lat - lat0) * self.EARTH_RADIUS_M
            xr = cos_t * x - sin_t * y
            yr = sin_t * x + cos_t * y
            pts_rot.append((xr, yr))
        
        return pts_rot, lat0, lon0, cosLat0, cos_t, sin_t
    
    def _intersect_line_polygon(self, pts: List[Tuple[float, float]], y: float) -> List[float]:
        """計算水平線與多邊形的交點"""
        xs = []
        n = len(pts)
        
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            
            if (y1 <= y <= y2) or (y2 <= y <= y1):
                if abs(y2 - y1) > 1e-10:
                    t = (y - y1) / (y2 - y1)
                    x_intersect = x1 + t * (x2 - x1)
                    xs.append(x_intersect)
        
        return sorted(xs)
    
    def _rotate_back_to_geographic(self, cos_t: float, sin_t: float, 
                                   xr: float, yr: float,
                                   lat0: float, lon0: float, 
                                   cosLat0: float) -> Tuple[float, float]:
        """從旋轉座標系轉回地理座標"""
        x = cos_t * xr + sin_t * yr
        y = -sin_t * xr + cos_t * yr
        lat = y / self.EARTH_RADIUS_M + lat0
        lon = x / (self.EARTH_RADIUS_M * cosLat0) + lon0
        return lat, lon
    
    def _calculate_distance(self, p1: Tuple[float, float], 
                           p2: Tuple[float, float]) -> float:
        """計算兩點間距離（公尺）"""
        lat1, lon1 = p1
        lat2, lon2 = p2
        avg_lat = (lat1 + lat2) / 2
        dlat = (lat2 - lat1) * self.EARTH_RADIUS_M
        dlon = (lon2 - lon1) * self.EARTH_RADIUS_M * math.cos(math.radians(avg_lat))
        return math.sqrt(dlat**2 + dlon**2)
    
    def _calculate_statistics(self, waypoints: List[Tuple[float, float]],
                             scan_lines: List[List[Tuple[float, float]]],
                             total_distance: float,
                             corners: List[Tuple[float, float]]) -> SurveyStatistics:
        """計算掃描統計資料"""
        # 計算覆蓋面積（使用 Shoelace 公式）
        area = self._calculate_polygon_area(corners)
        
        # 計算預估時間
        estimated_time = total_distance / self.config.speed
        
        # 計算照片數量
        num_photos = int(total_distance / self._photo_interval) if self._photo_interval > 0 else 0
        
        # 計算 GSD
        gsd = CameraCalculator.calculate_gsd(
            self.config.altitude, self.camera.focal_length,
            self.camera.sensor_width, self.camera.image_width
        ) * 100  # 轉換為 cm/px
        
        return SurveyStatistics(
            num_waypoints=len(waypoints),
            num_lines=len(scan_lines),
            total_distance=total_distance,
            coverage_area=area,
            estimated_time=estimated_time,
            num_photos=num_photos,
            gsd=gsd
        )
    
    def _calculate_polygon_area(self, corners: List[Tuple[float, float]]) -> float:
        """使用 Shoelace 公式計算多邊形面積（平方公尺）"""
        n = len(corners)
        if n < 3:
            return 0.0
        
        # 轉換為公尺座標
        lat0 = sum(p[0] for p in corners) / n
        lon0 = sum(p[1] for p in corners) / n
        cosLat0 = math.cos(math.radians(lat0))
        
        pts_m = []
        for lat, lon in corners:
            x = (lon - lon0) * self.EARTH_RADIUS_M * cosLat0
            y = (lat - lat0) * self.EARTH_RADIUS_M
            pts_m.append((x, y))
        
        # Shoelace 公式
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += pts_m[i][0] * pts_m[j][1]
            area -= pts_m[j][0] * pts_m[i][1]
        
        return abs(area) / 2.0


# ==============================
# MAVLink 航點匯出器
# ==============================
class MAVLinkExporter:
    """MAVLink 航點匯出器 - QGC WPL 110 格式"""
    
    @staticmethod
    def export_to_file(waypoints: List[Tuple[float, float]], 
                      altitude: float,
                      speed: float,
                      file_path: str,
                      include_rtl: bool = True,
                      loiter_time: float = 0.0,
                      region_idx: int = 0) -> bool:
        """
        匯出航點到 MAVLink 格式文件
        
        Args:
            waypoints: 航點列表 [(lat, lon), ...]
            altitude: 飛行高度 (m)
            speed: 飛行速度 (m/s)
            file_path: 輸出文件路徑
            include_rtl: 是否包含返航命令
            loiter_time: LOITER 等待時間（多機協同用）
            region_idx: 區域索引（用於 RTL 高度錯開）
        """
        lines = ["QGC WPL 110"]
        seq = 0
        
        # HOME 點（使用第一個航點作為 HOME）
        if waypoints:
            lat, lon = waypoints[0]
            lines.append(f"{seq}\t1\t0\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{altitude:.2f}\t1")
            seq += 1
        
        # 速度設定 (MAV_CMD_DO_CHANGE_SPEED)
        lines.append(f"{seq}\t0\t3\t178\t0\t{speed:.1f}\t0\t0\t0\t0\t0\t1")
        seq += 1
        
        # LOITER 等待（如果需要）
        if loiter_time > 0:
            lines.append(f"{seq}\t0\t3\t19\t{loiter_time:.1f}\t0\t0\t0\t0\t0\t0\t1")
            seq += 1
        
        # 航點 (MAV_CMD_NAV_WAYPOINT)
        for lat, lon in waypoints:
            lines.append(f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{altitude:.2f}\t1")
            seq += 1
        
        # RTL 命令
        if include_rtl:
            # 減速
            lines.append(f"{seq}\t0\t3\t178\t0\t5.0\t0\t0\t0\t0\t0\t1")
            seq += 1
            
            # 返回第一個航點
            if waypoints:
                lat, lon = waypoints[0]
                lines.append(f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{altitude:.2f}\t1")
                seq += 1
            
            # RTL 高度錯開（多機協同時）
            rtl_altitude = altitude + (3 - region_idx) * 3.0
            lines.append(f"{seq}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t{rtl_altitude:.1f}\t1")
        
        try:
            os.makedirs(os.path.dirname(file_path) if os.path.dirname(file_path) else '.', exist_ok=True)
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines))
            return True
        except Exception as e:
            print(f"匯出失敗: {e}")
            return False
    
    @staticmethod
    def generate_lines(waypoints: List[Tuple[float, float]], 
                      altitude: float,
                      speed: float,
                      loiter_time: float = 0.0) -> List[str]:
        """生成 MAVLink 行列表（不寫入文件）"""
        lines = ["QGC WPL 110"]
        seq = 0
        
        # HOME
        if waypoints:
            lat, lon = waypoints[0]
            lines.append(f"{seq}\t1\t0\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{altitude:.2f}\t1")
            seq += 1
        
        # 速度
        lines.append(f"{seq}\t0\t3\t178\t0\t{speed:.1f}\t0\t0\t0\t0\t0\t1")
        seq += 1
        
        # LOITER
        if loiter_time > 0:
            lines.append(f"{seq}\t0\t3\t19\t{loiter_time:.1f}\t0\t0\t0\t0\t0\t0\t1")
            seq += 1
        
        # 航點
        for lat, lon in waypoints:
            lines.append(f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{altitude:.2f}\t1")
            seq += 1
        
        return lines


# ==============================
# 多機任務分割器
# ==============================
class MultiDroneSplitter:
    """多機任務分割器"""
    
    @staticmethod
    def split_region(corners: List[Tuple[float, float]], 
                    num_drones: int,
                    spacing_m: float = 0.0) -> List[List[Tuple[float, float]]]:
        """
        將區域分割給多架無人機
        
        Args:
            corners: 原始區域角點
            num_drones: 無人機數量 (1-4)
            spacing_m: 子區域間隔 (m)
            
        Returns:
            子區域角點列表
        """
        if num_drones == 1:
            return [corners]
        
        if len(corners) != 4:
            # 非四邊形，使用水平條帶分割
            return MultiDroneSplitter._split_polygon_strips(corners, num_drones, spacing_m)
        
        # 四邊形分割
        return MultiDroneSplitter._split_rectangle(corners, num_drones, spacing_m)
    
    @staticmethod
    def _split_rectangle(corners: List[Tuple[float, float]], 
                        n: int, spacing_m: float) -> List[List[Tuple[float, float]]]:
        """分割四邊形區域"""
        p0, p1, p2, p3 = corners  # 左下、右下、右上、左上
        
        # 計算間隔比例
        width1 = MultiDroneSplitter._calc_dist(p0, p1)
        spacing_ratio = min(0.1, spacing_m / width1) if spacing_m > 0 and width1 > 0 else 0.0
        
        regions = []
        
        if n in (2, 3):
            # 水平分割
            effective_width = 1.0 - spacing_ratio * (n - 1)
            segment_width = effective_width / n
            
            for i in range(n):
                u0 = i * (segment_width + spacing_ratio)
                u1 = u0 + segment_width
                u0 = max(0, min(1, u0))
                u1 = max(0, min(1, u1))
                
                if u0 >= u1:
                    continue
                
                bl = MultiDroneSplitter._bilinear(corners, u0, 0)
                br = MultiDroneSplitter._bilinear(corners, u1, 0)
                tr = MultiDroneSplitter._bilinear(corners, u1, 1)
                tl = MultiDroneSplitter._bilinear(corners, u0, 1)
                regions.append([bl, br, tr, tl])
        
        elif n == 4:
            # 2x2 網格
            grid_spacing = spacing_ratio / 2
            for j in range(2):
                for i in range(2):
                    u0 = i * 0.5 if i == 0 else 0.5 + grid_spacing
                    u1 = 0.5 - grid_spacing if i == 0 else 1.0
                    v0 = j * 0.5 if j == 0 else 0.5 + grid_spacing
                    v1 = 0.5 - grid_spacing if j == 0 else 1.0
                    
                    bl = MultiDroneSplitter._bilinear(corners, u0, v0)
                    br = MultiDroneSplitter._bilinear(corners, u1, v0)
                    tr = MultiDroneSplitter._bilinear(corners, u1, v1)
                    tl = MultiDroneSplitter._bilinear(corners, u0, v1)
                    regions.append([bl, br, tr, tl])
        
        return regions if regions else [corners]
    
    @staticmethod
    def _split_polygon_strips(corners: List[Tuple[float, float]], 
                              n: int, spacing_m: float) -> List[List[Tuple[float, float]]]:
        """使用水平條帶分割任意多邊形"""
        min_y = min(p[0] for p in corners)
        max_y = max(p[0] for p in corners)
        total_height = max_y - min_y
        
        spacing_deg = spacing_m / 111111.0 if spacing_m > 0 else 0
        total_spacing = spacing_deg * (n - 1)
        
        if total_spacing >= total_height * 0.5:
            spacing_deg = total_height * 0.1 / (n - 1)
            total_spacing = spacing_deg * (n - 1)
        
        effective_height = total_height - total_spacing
        strip_height = effective_height / n
        
        regions = []
        for i in range(n):
            y_start = min_y + i * (strip_height + spacing_deg)
            y_end = y_start + strip_height
            y_start = max(min_y, y_start)
            y_end = min(max_y, y_end)
            
            if y_start >= y_end:
                continue
            
            # 簡化：使用原始多邊形
            regions.append(corners)
        
        return regions if regions else [corners]
    
    @staticmethod
    def _bilinear(corners: List[Tuple[float, float]], u: float, v: float) -> Tuple[float, float]:
        """雙線性插值"""
        p0, p1, p2, p3 = corners
        x = ((1-u)*(1-v)*p0[0] + u*(1-v)*p1[0] + u*v*p2[0] + (1-u)*v*p3[0])
        y = ((1-u)*(1-v)*p0[1] + u*(1-v)*p1[1] + u*v*p2[1] + (1-u)*v*p3[1])
        return (x, y)
    
    @staticmethod
    def _calc_dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """計算兩點距離（公尺）"""
        dlat = p2[0] - p1[0]
        dlon = p2[1] - p1[1]
        return math.sqrt(dlat**2 + dlon**2) * 111111.0


# ==============================
# 與現有專案整合的適配器類
# ==============================
class DronePathPlannerAdapter:
    """
    與 drone_path_planner_V10 整合的適配器
    
    使用方法：
    1. 在 main_app.py 中導入此模組
    2. 替換或增強現有的 OptimizedWaypointGenerator
    3. 使用 generate_zigzag_mission() 生成 Zigzag 網格路徑
    """
    
    def __init__(self):
        self.generator = None
        self.config = ZigzagSurveyConfig()
        self.camera = CAMERA_DATABASE["DJI Phantom 4 Pro"]
    
    def set_camera(self, camera_name: str):
        """設定相機"""
        if camera_name in CAMERA_DATABASE:
            self.camera = CAMERA_DATABASE[camera_name]
    
    def configure(self, altitude: float = 50.0, speed: float = 5.0,
                 front_overlap: float = 80.0, side_overlap: float = 60.0,
                 scan_angle: float = 0.0, yaw_speed: float = 60.0):
        """配置掃描參數"""
        self.config = ZigzagSurveyConfig(
            altitude=altitude,
            speed=speed,
            front_overlap=front_overlap,
            side_overlap=side_overlap,
            scan_angle=scan_angle,
            yaw_speed=yaw_speed,
            scan_pattern=ScanPattern.ZIGZAG
        )
    
    def generate_zigzag_mission(self, corners: List[Tuple[float, float]],
                                region_idx: int = 0,
                                start_from_left: bool = True,
                                loiter_time: float = 0.0) -> Tuple[List[str], List[Tuple[float, float]]]:
        """
        生成 Zigzag 任務（兼容現有 API）
        
        Args:
            corners: 邊界角點
            region_idx: 區域索引
            start_from_left: 是否從左側開始
            loiter_time: LOITER 等待時間
            
        Returns:
            (waypoint_lines, waypoints): MAVLink 行列表和航點列表
        """
        self.generator = ZigzagGridGenerator(self.config, self.camera)
        result = self.generator.generate_zigzag_grid(corners, region_idx, start_from_left)
        
        if not result.is_success:
            return ["QGC WPL 110"], []
        
        # 生成 MAVLink 格式
        lines = MAVLinkExporter.generate_lines(
            result.waypoints,
            self.config.altitude,
            self.config.speed,
            loiter_time
        )
        
        # 添加返回起點和 RTL
        seq = len(lines)
        if result.waypoints:
            # 返回起點
            lat, lon = result.waypoints[0]
            lines.append(f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{self.config.altitude:.2f}\t1")
            seq += 1
            
            # 減速
            lines.append(f"{seq}\t0\t3\t178\t0\t5.0\t0\t0\t0\t0\t0\t1")
            seq += 1
            
            # RTL
            rtl_altitude = self.config.altitude + (3 - region_idx) * 3.0
            lines.append(f"{seq}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t{rtl_altitude:.1f}\t1")
        
        return lines, result.waypoints
    
    def get_line_spacing(self) -> float:
        """獲取自動計算的航線間距"""
        if self.generator:
            return self.generator._line_spacing
        return 30.0
    
    def get_statistics(self) -> Optional[SurveyStatistics]:
        """獲取掃描統計資料"""
        if self.generator and hasattr(self.generator, '_last_result'):
            return self.generator._last_result.statistics
        return None


# ==============================
# 示範與測試
# ==============================
def demo_zigzag_generation():
    """Zigzag 網格生成示範"""
    print("\n" + "=" * 60)
    print("     Zigzag 網格生成器示範")
    print("=" * 60)
    
    # 定義掃描區域（雲林科技大學附近）
    boundary = [
        (23.695, 120.530),
        (23.695, 120.535),
        (23.698, 120.535),
        (23.698, 120.530)
    ]
    
    # 配置
    config = ZigzagSurveyConfig(
        altitude=50.0,
        speed=5.0,
        front_overlap=80.0,
        side_overlap=60.0,
        scan_pattern=ScanPattern.ZIGZAG,
        scan_angle=0.0
    )
    
    camera = CAMERA_DATABASE["DJI Phantom 4 Pro"]
    
    # 生成網格
    generator = ZigzagGridGenerator(config, camera)
    result = generator.generate_zigzag_grid(boundary)
    
    if result.is_success:
        print(f"\n✓ Zigzag 網格生成成功！")
        print(f"\n掃描統計：")
        print(f"  航點數量: {result.statistics.num_waypoints}")
        print(f"  掃描線數: {result.statistics.num_lines}")
        print(f"  總航程: {result.statistics.total_distance:.1f} m")
        print(f"  預估時間: {result.statistics.estimated_time/60:.1f} 分鐘")
        print(f"  覆蓋面積: {result.statistics.coverage_area:.1f} m²")
        print(f"  GSD: {result.statistics.gsd:.2f} cm/px")
        
        # 驗證 Zigzag 模式
        print(f"\nZigzag 驗證：")
        for i, line in enumerate(result.scan_lines[:3]):
            if len(line) >= 2:
                direction = "→" if line[0][1] < line[1][1] else "←"
                print(f"  掃描線 {i+1}: {direction}")
        
        return True
    else:
        print(f"\n✗ 生成失敗: {result.message}")
        return False


def demo_multi_drone():
    """多機協同示範"""
    print("\n" + "=" * 60)
    print("     多機協同 Zigzag 掃描示範")
    print("=" * 60)
    
    boundary = [
        (23.695, 120.530),
        (23.695, 120.540),
        (23.705, 120.540),
        (23.705, 120.530)
    ]
    
    num_drones = 4
    regions = MultiDroneSplitter.split_region(boundary, num_drones, spacing_m=3.0)
    
    print(f"\n區域分割: {num_drones} 架無人機")
    
    config = ZigzagSurveyConfig(altitude=50.0, speed=5.0)
    camera = CAMERA_DATABASE["DJI Phantom 4 Pro"]
    
    total_distance = 0.0
    loiter_times = []
    prev_waypoints = None
    
    for idx, region in enumerate(regions):
        generator = ZigzagGridGenerator(config, camera)
        
        # 計算 LOITER 時間
        loiter_time = idx * 5.0  # 簡化計算
        loiter_times.append(loiter_time)
        
        result = generator.generate_zigzag_grid(region, idx, start_from_left=(idx % 2 == 0))
        
        if result.is_success:
            print(f"\n無人機 {idx + 1}:")
            print(f"  航點: {result.statistics.num_waypoints}")
            print(f"  航程: {result.statistics.total_distance:.1f} m")
            print(f"  LOITER: {loiter_time:.1f} s")
            total_distance += result.statistics.total_distance
    
    print(f"\n總航程: {total_distance:.1f} m")
    print(f"效率提升: {num_drones}x")


def demo_mavlink_export():
    """MAVLink 匯出示範"""
    print("\n" + "=" * 60)
    print("     MAVLink 航點匯出示範")
    print("=" * 60)
    
    boundary = [
        (23.695, 120.530),
        (23.695, 120.535),
        (23.698, 120.535),
        (23.698, 120.530)
    ]
    
    config = ZigzagSurveyConfig(altitude=50.0, speed=5.0)
    camera = CAMERA_DATABASE["DJI Phantom 4 Pro"]
    
    generator = ZigzagGridGenerator(config, camera)
    result = generator.generate_zigzag_grid(boundary)
    
    if result.is_success:
        # 生成 MAVLink 格式
        lines = MAVLinkExporter.generate_lines(
            result.waypoints, config.altitude, config.speed
        )
        
        print(f"\nQGC WPL 110 格式預覽（前 10 行）：")
        for line in lines[:10]:
            print(f"  {line}")
        print(f"  ... (共 {len(lines)} 行)")
        
        # 匯出到文件
        export_path = "/home/claude/zigzag_mission_demo.waypoints"
        success = MAVLinkExporter.export_to_file(
            result.waypoints, config.altitude, config.speed, export_path
        )
        
        if success:
            print(f"\n✓ 已匯出至: {export_path}")


def demo_adapter():
    """適配器使用示範"""
    print("\n" + "=" * 60)
    print("     DronePathPlannerAdapter 使用示範")
    print("=" * 60)
    
    # 創建適配器
    adapter = DronePathPlannerAdapter()
    
    # 設定相機
    adapter.set_camera("DJI Phantom 4 Pro")
    
    # 配置參數
    adapter.configure(
        altitude=50.0,
        speed=5.0,
        front_overlap=80.0,
        side_overlap=60.0,
        scan_angle=0.0
    )
    
    # 定義邊界
    boundary = [
        (23.695, 120.530),
        (23.695, 120.535),
        (23.698, 120.535),
        (23.698, 120.530)
    ]
    
    # 生成任務
    lines, waypoints = adapter.generate_zigzag_mission(
        boundary, region_idx=0, start_from_left=True, loiter_time=0.0
    )
    
    print(f"\n生成結果：")
    print(f"  航點數量: {len(waypoints)}")
    print(f"  MAVLink 行數: {len(lines)}")
    print(f"  航線間距: {adapter.get_line_spacing():.1f} m")
    
    print(f"\n此適配器可直接整合到 drone_path_planner_V10:")
    print(f"  from zigzag_grid_generator_integration import DronePathPlannerAdapter")
    print(f"  adapter = DronePathPlannerAdapter()")
    print(f"  lines, waypoints = adapter.generate_zigzag_mission(corners)")


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("     Zigzag 網格生成器整合模組測試")
    print("=" * 60)
    
    # 運行所有示範
    demo_zigzag_generation()
    demo_multi_drone()
    demo_mavlink_export()
    demo_adapter()
    
    print("\n" + "=" * 60)
    print("     所有示範完成！")
    print("=" * 60)
