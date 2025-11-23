from dataclasses import dataclass

@dataclass
class TouristState:
    """游客状态对象"""
    time: float          # 时间
    velocity: float      # 线速度
    angular_velocity: float  # 角速度
    angle: float        # 角度 (弧度)
    height: float 
    accCentripetal: float # 向心加速度
    accTangential: float  # 切向加速度
    force_tangential: float = 0.0  # 支持力切向分量（面前方向）
    force_normal: float = 0.0      # 支持力法向分量（头顶方向）


