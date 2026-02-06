"""
전역 설정 관리

[설정 구조]
├── Config: 시스템 전체 설정
│   ├── Lidar: LiDAR 센서 관련 설정
│   ├── Terrain: 지형 분석 및 Costmap 설정
│   ├── ASTAR: A* 경로 계획 설정
│   ├── PID: PID 제어기 설정
│   ├── DWA: Dynamic Window Approach 설정
│   ├── Stuck: Stuck 감지 설정
│   ├── Recovery: Stuck 복구 설정
│   └── SEQ4: SEQ 4 재계획 설정
│
├── SEQ 2 전투 시스템 설정
│   ├── PitchConfig: Pitch 제어 설정
│   ├── SmoothingConfig: 조준 스무싱 설정
│   ├── OutputHoldConfig: 출력 유지 설정
│   ├── FusionConfig: LiDAR-이미지 퓨전 설정
│   ├── AimingConfig: 조준 및 발사 임계값
│   ├── TurretConfig: 포탑 회전 설정
│   ├── OffsetConfig: Lead aiming 오프셋 설정
│   ├── TargetLockConfig: 타겟 잠금 설정
│   ├── StateMachineConfig: 전투 모드 State Machine
│   ├── OverlayConfig: 오버레이 렌더링 설정
│   ├── CameraConfig: 카메라 설정
│   └── CombatSystemConfig: 객체 인식 설정
│
└── 전역 객체 인스턴스 (app.py에서 직접 사용)
"""

import math
import os
from dataclasses import dataclass, field
from typing import Optional
import platform

# ==============================================================================
# 시스템 전체 설정
# ==============================================================================

class Config:
    """시스템 전체 설정 클래스"""
    
    # 월드 범위 (Unity 맵 크기)
    WORLD_MIN_XZ = 0.0
    WORLD_MAX_XZ = 300.0
    
    # ========================================
    # LiDAR 설정
    # ========================================
    class Lidar:
        """LiDAR 센서 관련 설정"""

        # LiDAR 데이터 폴더 경로
        def _set_lidar_folder(user :str = "acorn"):
            os_name = platform.system()

            # 탐색할 하위 경로 후보들 (중요도 순서대로)
            sub_path_candidates = [
                ["OneDrive", "문서", "Tank Challenge", "lidar_data"],
                ["OneDrive", "Documents", "Tank Challenge", "lidar_data"],
                ["Documents", "Tank Challenge", "lidar_data"],
                ["문서", "Tank Challenge", "lidar_data"],
                ["My documents", "Tank Challenge", 'lidar_data']
            ]

            if os_name == "Windows":
                base_path = f"C:\\Users\\{user}"
            else:
                base_path = f"/mnt/c/Users/{user}"

            for sub in sub_path_candidates:
                full_path = os.path.join(base_path, *sub)
                if os.path.exists(full_path): return full_path
            
            print("LiDAR 폴더 설정 완료!")

        # 예시 LIDAR_FOLDER = set_lidar_folder(user="acorn")
        LIDAR_FOLDER = _set_lidar_folder(user="soeao")
        LIDAR_FILE_PATTERN = "*.json"
        
        # 모니터링 설정
        MONITOR_INTERVAL = 0.1  # 모니터링 간격 (초)
        LIDAR_MAX_RETRIES = 5   # Windows 파일 잠금 대응 재시도 횟수
        
        # 로컬 맵 범위
        LOCAL_RADIUS = 30.0
        
        # 그리드 설정
        GRID_SIZE = 1


    # ========================================
    # 지형 필터링 + Costmap 생성 설정
    # ========================================
    class Terrain:
        """지형 분석 및 Costmap 생성 설정"""
        
        # 전차 반경 (장애물 거리 비용 계산 기준)
        TANK_RADIUS = 2.5
        
        # 장애물 영향 반경
        INFLATION_RADIUS = 2.0  # tank_radius보다 클 것
        COSTMAP_INFLATION = 1   # Costmap 장애물 팽창 반경 (셀 단위)
        
        # 지형 분석 임계값
        SLOPE_THRESH = 45.0        # 경사 각도 임계값 (도)
        PLANE_THRESH = 0.15        # 평면 거리 임계값
        GROUND_RATIO_TH = 0.7      # 지면 비율 임계값
        HEIGHT_STD_TH = 0.1        # 높이 표준편차 임계값
        MIN_PTS_CLASSIFY = 1       # 분류 최소 포인트 수
        OBSTACLE_HEIGHT_TH = 0.5   # 장애물 높이 임계값
        MAX_STEP_HEIGHT = 0.8      # 최대 계단 높이
        
        # 지형 비용 가중치 (합계 = 1.0)
        W_SLOPE = 0.20   # 경사 가중치
        W_ROUGH = 0.35   # 거칠기 가중치
        W_GROUND = 0.45  # 지면 가중치
        
        # Rough cost 정규화 분모
        ROUGH_STD_NORM = 0.9
        
        # 비용 기본값
        UNKNOWN_COST = 0.2        # 미탐색 셀 기본 비용
        FILL_TERRAIN_COST = 0.5   # 비어있는 Terrain cost 채우기
        FILL_FINAL_COST = 0.2     # 비어있는 Final cost 채우기
    
    # ========================================
    # A* 경로 계획 설정
    # ========================================
    class ASTAR:
        """A* 경로 계획 알고리즘 설정"""
        
        # A* 이동 비용
        COST_STRAIGHT = 10  # 직선 이동 비용
        COST_DIAGONAL = 13  # 대각선 이동 비용 (√2 * 10 ≈ 14, 약간 작게 설정)
        
        # 목적지 스냅 탐색 반경
        SNAP_RADIUS = 20
        
        # 장애물 버퍼 (SEQ별로 다름)
        OBSTACLE_MARGIN = 3.3
        OBSTACLE_MARGIN_SEQ1 = 3.0
        OBSTACLE_MARGIN_SEQ3 = 2.3
        OBSTACLE_MARGIN_SEQ4 = 1.5
        
        # 경로 계획에 사용할 셀 크기
        CELL_SIZE = 1.0
        
        # ========================================
        # 장애물 회피 강화 설정 (Proximity Cost)
        # ========================================
        # 안전성 가중치: 높을수록 장애물에서 멀리 떨어진 경로 선호
        # - 0: 순수 최단 경로 (기존 A*와 동일)
        # - 1~2: 적당한 안전성 (권장)
        # - 3+: 매우 안전한 경로 (우회가 많아질 수 있음)
        SAFETY_WEIGHT = 1.5
        SAFETY_WEIGHT_SEQ1 = 2.0   # SEQ1: 좀 더 안전하게
        SAFETY_WEIGHT_SEQ3 = 1.5   # SEQ3: 적당히
        SAFETY_WEIGHT_SEQ4 = 1.0   # SEQ4: 빠른 경로 우선
        
        # 장애물 영향 반경: 이 거리 내의 셀들은 장애물과의 거리에 따라 추가 비용 부과
        PROXIMITY_RADIUS = 8.0      # 기본 영향 반경 (m)
        PROXIMITY_RADIUS_SEQ1 = 10.0  # SEQ1: 넓은 영향 범위
        PROXIMITY_RADIUS_SEQ3 = 8.0   # SEQ3: 적당히
        PROXIMITY_RADIUS_SEQ4 = 6.0   # SEQ4: 좁은 영향 범위
        
        @staticmethod
        def get_obstacle_margin(seq):
            """
            SEQ에 따른 장애물 마진 반환
            
            Args:
                seq: 현재 SEQ 번호 (1, 3, 4)
            
            Returns:
                float: 해당 SEQ의 장애물 마진
            """
            if seq == 1:
                return Config.ASTAR.OBSTACLE_MARGIN_SEQ1
            elif seq == 3:
                return Config.ASTAR.OBSTACLE_MARGIN_SEQ3
            elif seq == 4:
                return Config.ASTAR.OBSTACLE_MARGIN_SEQ4
            return Config.ASTAR.OBSTACLE_MARGIN
        
        @staticmethod
        def get_safety_weight(seq):
            """
            SEQ에 따른 안전성 가중치 반환
            
            Args:
                seq: 현재 SEQ 번호 (1, 3, 4)
            
            Returns:
                float: 해당 SEQ의 안전성 가중치
            """
            if seq == 1:
                return Config.ASTAR.SAFETY_WEIGHT_SEQ1
            elif seq == 3:
                return Config.ASTAR.SAFETY_WEIGHT_SEQ3
            elif seq == 4:
                return Config.ASTAR.SAFETY_WEIGHT_SEQ4
            return Config.ASTAR.SAFETY_WEIGHT
        
        @staticmethod
        def get_proximity_radius(seq):
            """
            SEQ에 따른 장애물 영향 반경 반환
            
            Args:
                seq: 현재 SEQ 번호 (1, 3, 4)
            
            Returns:
                float: 해당 SEQ의 장애물 영향 반경
            """
            if seq == 1:
                return Config.ASTAR.PROXIMITY_RADIUS_SEQ1
            elif seq == 3:
                return Config.ASTAR.PROXIMITY_RADIUS_SEQ3
            elif seq == 4:
                return Config.ASTAR.PROXIMITY_RADIUS_SEQ4
            return Config.ASTAR.PROXIMITY_RADIUS
    
    # ========================================
    # PID 제어 설정
    # ========================================
    class PID:
        """PID 제어기 설정"""
        
        # PID 계수
        KP = 0.02    # 비례 계수
        KI = 0.0001  # 적분 계수
        KD = 0.01    # 미분 계수
        
        # 속도 제어 파라미터
        MAX_SPEED_WEIGHT = 0.5   # 직진 시 최고 속도 가중치
        MIN_SPEED_WEIGHT = 0.25  # 조향 많을 때 유지할 최소 속도
        SPEED_REDUCT_GAIN = 1.0  # 조향에 따른 감속 민감도
        
        # 조향 제어 파라미터
        STEER_SENSITIVITY = 0.7  # 조향 민감도
        ERROR_THRESHOLD = 30.0   # 오차 임계값 (도)
        ERROR_RANGE = 45         # 감속을 위한 추가 오차 범위 (도)
    
    # ========================================
    # DWA (Dynamic Window Approach) 설정
    # ========================================
    class DWA:
        
        # 속도 제한 (적당한 속도 + 충분한 가속)
        MAX_SPEED = 0.15           # 최대 속도 (적당히)
        MIN_SPEED = -0.10          # 최소 속도 (후진)
        MAX_YAW_RATE = 60.0 * math.pi / 180.0  # 최대 회전 속도 (60°/s)
        
        # 가속도 제한 (충분히 높게 - Dynamic Window 넓게)
        MAX_ACCEL = 0.5            # 최대 가속도 (넉넉하게)
        MAX_DELTA_YAW_RATE = 90.0 * math.pi / 180.0  # 회전 가속도 (90°/s²)
        
        # 탐색 해상도
        V_RESOLUTION = 0.02        # 속도 탐색 해상도 (0.01 → 0.02)
        YAW_RATE_RESOLUTION = 5.0 * math.pi / 180.0  # 회전 해상도 (5°)
        
        # 예측 파라미터
        DT = 0.1                   # 시뮬레이션 시간 간격 (초)
        PREDICT_TIME = 2.0         # 예측 시간 (초) - 약간 단축
        
        # 비용 함수 가중치 (균형 잡힌 설정)
        TO_GOAL_COST_GAIN = 0.15   # 목표 지향 비용 (약간 증가)
        SPEED_COST_GAIN = 0.3      # 🔧 속도 비용 증가 (빠른 속도 선호)
        OBSTACLE_COST_GAIN = 6.0   # 장애물 회피 비용
        
        # 측면 탈출 보너스 (stuck 감지 시 측면 경로에 보너스)
        LATERAL_ESCAPE_GAIN = 3.0  # 측면(±45~135°) 방향 경로에 보너스
        
        # 로봇 파라미터
        ROBOT_RADIUS = 4.0         # 로봇 반경 (안전 영역)
        ROBOT_STUCK_FLAG_CONS = 0.001  # Stuck 판단 임계값
        
        # 조향 페널티 (거의 없음 - 회전 적극 허용)
        STEERING_PENALTY = 0.05    # 급격한 조향 페널티 (거의 무시)
        
        # 장애물 거리 임계값 (SEQ 4 가상 라이다용)
        COLLISION_DISTANCE = 1.5   # 충돌 거리 (이내면 경로 무효)
        DANGER_DISTANCE = 4.0      # 위험 거리 (높은 비용)
        SAFE_DISTANCE = 8.0        # 안전 거리 (낮은 비용)
    
    # ========================================
    # Stuck 감지 설정
    # ========================================
    class Stuck:
        """Stuck 감지 설정"""
        
        STUCK_THRESHOLD = 0.3  # 이동 거리 임계값 (m)
        STUCK_COUNT_LIMIT = 15  # 🔧 Stuck 카운터 임계값 (10 → 15, 좀 더 기다림)
    
    # ========================================
    # Stuck 복구 설정
    # ========================================
    class Recovery:
        """Stuck 복구 설정"""
        
        # 복구 동작 시간 (초)
        PHASE1_SEC = 1.5  # 후진 + 회전 시간
        PHASE2_SEC = 1.0  # 제자리 회전 시간
        
        # 복구 동작 가중치
        PHASE1_WS_WEIGHT = 0.6  # 후진 가중치
        PHASE1_AD_WEIGHT = 0.5  # 회전 가중치 (🔧 증가: 0.4 → 0.5)
        PHASE2_AD_WEIGHT = 0.7  # 제자리 회전 가중치
        
        # 복구 속도
        REVERSE_SPEED = 0.1  # 후진 속도
    
    # ========================================
    # 경로 추종 설정
    # ========================================
    LOOKAHEAD_DIST = 8.0      # 전방 주시 거리 (m) - SEQ 1, 3용
    ARRIVAL_THRESHOLD = 7.0   # 도착 판단 거리 (m)
    MIN_TARGET_DIST = 2.0     # 최소 타겟 거리 (m)
    
    # ========================================
    # SEQ 4 재계획 설정
    # ========================================
    class SEQ4:
        """SEQ 4 자율주행 재계획 설정"""
        
        # SEQ 4 전용 LOOKAHEAD
        LOOKAHEAD_DIST = 10.0  # 전방 주시 거리 (m)
        
        # 재계획 트리거 모드
        REPLAN_MODE = "time"  # "distance" 또는 "time"
        
        # 거리 기반 재계획 (REPLAN_MODE = "distance")
        REPLAN_DISTANCE_INTERVAL = 50.0  # 진행 거리마다 재계획 (m)
        
        # 시간 기반 재계획 (REPLAN_MODE = "time")
        REPLAN_TIME_INTERVAL = 5.0  # 시간마다 재계획 (초)
        
        # 전차 주변 장애물 제외 반경
        ROBOT_CLEAR_RADIUS = 10.0  # A* 경로 생성 시 전차 주변 클리어 반경 (m)
        
        # 경로 최소 노드 수
        MIN_PATH_NODES = 5  # 이보다 적으면 재계획

        # 하이브리드 자율주행 모드 (A* + PPO 강화학습)
        HYBRID_MODE_ENABLED = True  # True: A* + PPO 혼합, False: PPO만 사용

        # 명령 혼합 가중치 (합계 = 1.0)
        ASTAR_WEIGHT = 0.75  # A* 명령 가중치 (0.0 ~ 1.0) - 경로 계획
        PPO_WEIGHT = 0.25    # PPO 명령 가중치 (0.0 ~ 1.0) - 장애물 회피

        # 권장: ASTAR_WEIGHT + PPO_WEIGHT = 1.0
        # 예시:
        #   - 0.9 + 0.1: A* 위주, PPO는 미세 조정만
        #   - 0.7 + 0.3: A*와 PPO 균형
        #   - 0.5 + 0.5: 동등한 영향력

        # PPO 실패 시 폴백
        PPO_FALLBACK_TO_ASTAR = True  # PPO 실패 시 A* 100%로 전환

    @staticmethod
    def clamp_world_xz(x: float, z: float, margin: float = 0.5):
        """
        월드 좌표 범위 제한
        
        Args:
            x: X 좌표
            z: Z 좌표
            margin: 경계로부터의 여유 거리
        
        Returns:
            tuple: (제한된 x, 제한된 z)
        """
        x_clamped = min(
            max(float(x), Config.WORLD_MIN_XZ + margin),
            Config.WORLD_MAX_XZ - margin
        )
        z_clamped = min(
            max(float(z), Config.WORLD_MIN_XZ + margin),
            Config.WORLD_MAX_XZ - margin
        )
        return x_clamped, z_clamped


# ==============================================================================
# SEQ 2 전투 시스템 설정
# ==============================================================================

@dataclass
class PitchConfig:
    """Pitch 제어 설정"""
    up_gain: float = 0.07              # 상승 게인
    weight_min: float = 0.01           # 최소 가중치
    weight_max: float = 0.03           # 최대 가중치
    cmd_interval_sec: float = 0.08     # 명령 간격 (초)
    cmd_min_deg: float = 0.3           # 최소 명령 각도 (도)
    disable_down: bool = True          # 하강 비활성화 여부
    down_ok_deg: float = 15.0          # 하강 허용 각도 (도)


@dataclass
class SmoothingConfig:
    """조준 스무싱 설정"""
    alpha: float = 0.2                 # 스무싱 계수 (0~1)
    stable_duration: float = 0.20      # 안정 판단 시간 (초)


@dataclass
class OutputHoldConfig:
    """출력 유지 설정"""
    duration_sec: float = 5.0          # 출력 유지 시간 (초)


@dataclass
class FusionConfig:
    """LiDAR-이미지 센서 퓨전 설정"""
    timeout_sec: float = 2.0           # 퓨전 타임아웃 (초)
    pose_timeout_sec: float = 2.0      # 포즈 타임아웃 (초)
    pose_buffer_maxlen: int = 1000     # 포즈 버퍼 최대 길이
    min_fuse_points: int = 2           # 최소 퓨전 포인트 수
    min_det_conf: float = 0.25         # 최소 탐지 신뢰도
    min_fire_conf: float = 0.40        # 최소 발사 신뢰도
    min_box_w: int = 10                # 최소 박스 너비 (픽셀)
    min_box_h: int = 10                # 최소 박스 높이 (픽셀)
    min_height_threshold: float = 5.0  # 높이 기반 필터링 임계값 (m)
    screen_margin: int = 200           # 화면 경계 마진 (픽셀)

@dataclass
class AimingConfig:
    """조준 및 발사 임계값 설정"""
    aim_yaw_thresh_deg: float = 3.0    # 조준 Yaw 임계값 (도)
    aim_pitch_thresh_deg: float = 2.0  # 조준 Pitch 임계값 (도)
    fire_yaw_thresh_deg: float = 3.0   # 발사 Yaw 임계값 (도)
    fire_pitch_thresh_deg: float = 6.0 # 발사 Pitch 임계값 (도)
    fire_min_dist: float = 2.0         # 최소 발사 거리 (m)
    fire_max_dist: float = 250.0       # 최대 발사 거리 (m)


@dataclass
class TurretConfig:
    """포탑 회전 설정"""
    yaw_offset_threshold: float = 1.0  # Yaw 오프셋 임계값
    body_yaw_thresh: float = 30.0      # 차체 회전 임계값 (도)
    turret_yaw_max: float = 45.0       # 포탑 최대 회전 각도 (도)
    body_rotation_weight: float = 0.1  # 차체 회전 가중치
    qe_weight: float = 0.05            # Q/E 키 가중치
    rf_weight: float = 0.02            # R/F 키 가중치


@dataclass
class OffsetConfig:
    """Lead aiming 오프셋 설정"""
    yaw_offset_deg: float = 5.0        # Yaw 오프셋 (도)
    aim_point_method: str = "weighted_center"  # 조준점 계산 방식
    pitch_offset_deg: float = 22.1     # Pitch 오프셋 (도)
    pitch_offset_min_dist: float = 80.0    # Pitch 오프셋 최소 거리 (m)
    pitch_offset_full_dist: float = 200.0  # Pitch 오프셋 최대 거리 (m)


@dataclass
class TargetLockConfig:
    """타겟 잠금 설정"""
    hold_sec: float = 5.0              # 잠금 유지 시간 (초)
    iou_thresh: float = 0.30           # IOU 임계값
    lost_grace_sec: float = 0.7        # 타겟 손실 유예 시간 (초)
    lock_delay: float = 0.6            # 락 확정 전 대기 시간 (초) - 이 시간 동안 타겟이 유지되어야 락을 검

    # LiDAR 기반 타겟 잠금
    enable_lidar_lock: bool = True     # LiDAR 잠금 활성화
    lock_duration: float = 10.0        # 잠금 지속 시간 (초)
    angle_tolerance: float = 3.0       # 각도 허용 오차 (도)
    distance_tolerance: float = 2.0    # 거리 허용 오차 (m)
    min_lidar_points: int = 2          # 최소 LiDAR 포인트 수
    update_interval: float = 0.1       # 업데이트 간격 (초)
    lock_on_fire_ready: bool = True    # 발사 준비 시 자동 잠금
    target_lost_distance_tolerance: float = 10.0  # 타겟 손실 거리 허용 오차 (m)
    
    # 센서 퓨전 ROI 파라미터
    roi_pad_px: int = 60               # ROI 확장 픽셀
    roi_conf: Optional[float] = None   # ROI YOLO Confidence
    roi_iou_th: float = 0.15           # IOU 임계값

@dataclass
class StateMachineConfig:
    """SEQ 2 전투 모드 State Machine 설정"""
    
    # SCAN 모드 설정
    scan_turret_speed: float = 0.04    # 터렛 회전 속도 (가중치)
    scan_direction: str = "E"          # 초기 스캔 방향 (E=우측, Q=좌측)
    scan_hold_sec: float = 5.0         # 스캔 유지 시간 (초)
    lowering_sec: float = 2.5
    
    # STANDBY 모드 설정
    target_lock_threshold_sec: float = 0.5   # 타겟 고정 대기 시간 (초)
    aim_alignment_yaw_thresh: float = 3.0    # Yaw 정렬 완료 임계값 (도)
    aim_alignment_pitch_thresh: float = 2.0  # Pitch 정렬 완료 임계값 (도)
    
    # FIRE 모드 설정
    fire_cooldown_sec: float = 0.5     # 발사 후 쿨다운 (초)
    post_fire_delay_sec: float = 1.0   # 발사 후 SEQ 전환 전 대기 (초)
    
    # 적 탐지 조건
    min_tanks_to_detect: int = 1       # 최소 탱크 수
    min_reds_to_detect: int = 2        # 최소 적군(Red) 수
    
    # Legacy 설정 (호환성 유지)
    wait_hit_timeout_sec: float = 2.0
    hit_detection_window_sec: float = 2.5
    retreat_sec: float = 3.0
    stop_after_first_hit: bool = True
    
    # RETREAT 정렬 완료 오차
    turret_alignment_threshold: float = 2.0


@dataclass
class OverlayConfig:
    """오버레이 렌더링 설정"""
    enable: bool = True                # 오버레이 활성화
    update_min_sec: float = 0.15       # 최소 업데이트 간격 (초)
    max_points: int = 6000             # 최대 포인트 수
    save_disk: bool = False            # 디스크 저장 여부


@dataclass
class CameraConfig:
    """카메라 설정"""
    h_fov_deg: float = 47.81           # 수평 FOV (도)
    v_fov_deg: float = 32.0            # 수직 FOV (도)


@dataclass
class CombatSystemConfig:
    """객체 인식 설정"""
    
    # YOLO 모델 경로
    model_path: str = "models/best.pt"  # Legacy 모델 (호환성 유지)
    model_cannon_path: str = "models/cannon.pt"     # Cannon 전용 모델
    model_integrated_path: str = "models/integrated.pt"  # 통합 객체 인식 모델

    # ONNX 모델 경로 (FP16 + 640 해상도)
    use_onnx: bool = True  # True: ONNX 사용, False: PyTorch 사용
    onnx_cannon_path: str = "models/cannon.onnx"      # Cannon 전용 ONNX 모델
    onnx_integrated_path: str = "models/integrated.onnx"  # 통합 객체 ONNX 모델
    onnx_input_size: int = 640  # ONNX 모델 입력 해상도
    onnx_fp16: bool = True  # FP16 모드 사용 여부
    onnx_use_gpu: bool = True  # GPU 가속 사용 여부
    
    # 파일 처리 설정
    file_poll_sec: float = 0.10        # 파일 폴링 간격 (초)
    max_sync_diff_sec: float = 1.0     # 최대 동기화 차이 (초)
    
    # 로깅 설정
    enable_http_log: bool = False
    enable_detect_log: bool = True
    enable_watch_log: bool = True
    
    # 객체 탐지 클래스 매핑 (듀얼 모델)
    map_cannon: dict = field(default_factory=lambda: {
        0: "Cannon"
    })
    map_integrated: dict = field(default_factory=lambda: {
        #0: "Mine",
        1: "Red",
        2: "Rock",
        3: "Tank",
        4: "Tree",
        })
    
    # 듀얼 모델 색상 설정
    color_cannon: str = "#00FF00"      # 녹색
    color_integrated: str = "#FF0000"  # 빨간색
    
    # 조건부 메시지 설정
    target_position_tank_threshold: int = 2
    target_position_red_threshold: int = 3
    target_position_message_duration: float = 3.0  # 초

    # 오버레이 폰트 설정
    overlay_font_size: int = 20      # 폰트 크기 (작게 설정)
    overlay_font_path: str = "arial.ttf" # 폰트 파일 경로 (시스템에 맞는 폰트 사용)

@dataclass 
class PrecisionAttackConfig:
    """정밀 조준(AUTO_ATTACK) 및 특정 좌표 조준 설정"""
    TARGET_YAW: float = 65.25
    TARGET_PITCH: float = 7.37
    TOLERANCE: float = 1.10        # 조준 완료 허용 오차 (도)
    TURRET_WEIGHT: float = 0.07    # 포탑 회전 속도 가중치
    # 포격 하드 코딩 값

# ==============================================================================
# 전역 객체 인스턴스
# app.py에서 '객체이름.변수이름' 형식으로 직접 사용 가능
# ==============================================================================

pitch_cfg = PitchConfig()
smooth_cfg = SmoothingConfig()
output_hold_cfg = OutputHoldConfig()
fusion_cfg = FusionConfig()
aim_cfg = AimingConfig()
turret_cfg = TurretConfig()
offset_cfg = OffsetConfig()
lock_cfg = TargetLockConfig()
sm_cfg = StateMachineConfig()
overlay_cfg = OverlayConfig()
camera_cfg = CameraConfig()
combat_config = CombatSystemConfig()
precision_cfg = PrecisionAttackConfig()