"""
UAV Path Planner
================

無人機路徑規劃系統 - 支援多旋翼和固定翼

主要功能：
- 網格掃描任務規劃 (Zigzag/Parallel/Spiral)
- A* 全局路徑規劃
- DWA 局部避障
- MAVLink 航點匯出

使用方式：
    from uav_path_planner import (
        GridSurveyGenerator, SurveyConfig,
        AStarPlanner, DWAPlanner,
        MultirotorModel, VehicleFactory
    )
"""

__version__ = "1.0.0"
__author__ = "UAV Path Planner Team"

# 核心模組導出
from .core.global_planner.grid_generator import (
    GridSurveyGenerator,
    SurveyConfig,
    ScanPattern,
    CameraConfig,
    SurveyStatistics
)

from .core.global_planner.astar import (
    AStarPlanner,
    AStarConfig
)

from .core.local_planner.dwa import (
    DWAPlanner,
    DWAConfig
)

from .core.base.vehicle_base import (
    VehicleModel,
    VehicleFactory,
    VehicleType,
    VehicleState,
    VehicleConstraints,
    VehicleConfig
)

from .core.vehicles.multirotor import (
    MultirotorModel
)

from .core.geometry.coordinate import (
    CoordinateTransformer,
    GeoPoint
)

from .core.geometry.polygon import (
    PolygonUtils
)

from .core.base.planner_base import (
    PlannerFactory,
    PlannerType,
    PlannerResult,
    PlannerStatus
)

__all__ = [
    # Grid Survey
    'GridSurveyGenerator',
    'SurveyConfig',
    'ScanPattern',
    'CameraConfig',
    'SurveyStatistics',
    
    # A* Planner
    'AStarPlanner',
    'AStarConfig',
    
    # DWA Planner
    'DWAPlanner',
    'DWAConfig',
    
    # Vehicle
    'VehicleModel',
    'VehicleFactory',
    'VehicleType',
    'VehicleState',
    'VehicleConstraints',
    'VehicleConfig',
    'MultirotorModel',
    
    # Geometry
    'CoordinateTransformer',
    'GeoPoint',
    'PolygonUtils',
    
    # Base
    'PlannerFactory',
    'PlannerType',
    'PlannerResult',
    'PlannerStatus',
]
