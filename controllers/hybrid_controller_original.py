"""
하이브리드 제어기 (A* + DWA + PID) - 개선 버전

주요 개선사항:
1. DWA 우선 사용 정책 - PID 폴백 조건 완화
2. 소프트 장애물 비용 (Soft Obstacle Cost) - inf 대신 점진적 비용
3. 적응적 DWA 파라미터 - 장애물 밀도에 따라 조정
4. DWA 실패 시 후진 + 회전 먼저 시도
"""
import math
import time
import numpy as np
from controllers.pid_controller import PIDController
from planners.astar_planner import AStarPlanner, ObstacleRect
from planners.dwa_planner import DWAConfig, motion_model, calc_dynamic_window
from planners.dwa_planner import predict_trajectory, calc_to_goal_cost
from utils.visualization import save_path_image


class ImprovedDWAConfig(DWAConfig):
    """개선된 DWA 설정 - 더 관대한 장애물 회피 + 전진 우선"""
    
    def __init__(self, config):
        super().__init__(config)
        
        # 🆕 소프트 장애물 비용 관련 설정 (대폭 완화)
        self.soft_obstacle_threshold = 0.8  # 더 관대
        self.hard_obstacle_threshold = 0.99  # 거의 확실한 장애물만 inf
        self.global_obstacle_cost = 5.0  # 전역 장애물 비용 최소화
        self.boundary_soft_margin = 1.0  # 경계 마진 축소
        
        # 🆕 적응적 DWA
        self.adaptive_mode = True
        self.min_predict_time = 1.0
        self.max_predict_time = 2.5  # 3.0 → 2.5로 단축


def calc_soft_costmap_cost(traj, costmap, origin, resolution, state_manager=None, dwa_config=None):
    """
    소프트 Costmap 비용 계산 - inf 대신 점진적 비용 부여
    
    개선점:
    1. 장애물 근처: 거리에 비례한 점진적 비용 (inf → soft penalty)
    2. 전역 장애물: 높은 비용이지만 inf는 아님
    3. 경계 근처: 소프트 마진으로 점진적 비용
    """
    from config import Config
    
    if dwa_config is None:
        soft_threshold = 0.6
        hard_threshold = 0.95
        global_obs_cost = 50.0
        boundary_margin = 2.0
    else:
        soft_threshold = dwa_config.soft_obstacle_threshold
        hard_threshold = dwa_config.hard_obstacle_threshold
        global_obs_cost = dwa_config.global_obstacle_cost
        boundary_margin = dwa_config.boundary_soft_margin

    total_cost = 0.0
    max_cell_cost = 0.0
    
    for i, state in enumerate(traj):
        if i < 5: continue

        x = state[0]
        z = state[1]

        # 1. 월드 경계 체크 - 소프트 마진 적용
        # 완전히 벗어나면 inf
        if (x <= Config.WORLD_MIN_XZ or x >= Config.WORLD_MAX_XZ or 
            z <= Config.WORLD_MIN_XZ or z >= Config.WORLD_MAX_XZ):
            return float("inf")
        
        # 2. 실시간 가상 라이다 체크
        if state_manager and hasattr(state_manager, 'is_global_obstacle'):
            if state_manager.is_global_obstacle(x, z):
                return float("inf") # 갈 길이 장애물인 경우만 차단
        
        # 3. Costmap 체크 - 소프트 비용
        if costmap is None or origin is None:
            continue

        ix = int((x - origin[0]) / resolution)
        iz = int((z - origin[1]) / resolution)

        # Costmap 범위 밖은 통과 가능
        if ix < 0 or iz < 0 or iz >= costmap.shape[0] or ix >= costmap.shape[1]:
            continue

        cell_cost = float(costmap[iz, ix])

        # 완전 장애물 (hard threshold)
        if cell_cost >= hard_threshold:
            return float("inf")
        
        # 위험 지역 (soft threshold ~ hard threshold)
        if cell_cost >= soft_threshold:
            # 점진적 비용: 0.8~0.99 → 2~20 (대폭 완화)
            normalized = (cell_cost - soft_threshold) / (hard_threshold - soft_threshold)
            soft_cost = 2.0 + normalized * 18.0
            total_cost += soft_cost

        if cell_cost > max_cell_cost:
            max_cell_cost = cell_cost
    
    return total_cost + max_cell_cost * 5.0  # 10.0 → 5.0

# 궤적의 각 지점에서 전역 장애물과의 거리를 계산하여 패널티 부과
def calc_virtual_lidar_cost(traj, state_manager, dwa_config):
    """
    Costmap 대신 전역 장애물 리스트를 기반으로 가상 LiDAR 거리 비용 계산
    """
    if not state_manager or not state_manager.global_obstacles:
        return 0.0

    total_cost = 0.0
    # 장애물 회피를 위한 임계값 설정
    safe_dist = 2.5  # 이 거리보다 가까우면 패널티 시작
    critical_dist = 1.0  # 이 거리보다 가까우면 충돌로 간주 (inf)

    for i, state in enumerate(traj):
        if i < 3: continue  # 초기 위치는 제외

        x, z = state[0], state[1]
        
        # 1. 월드 경계 체크 (기존 로직 유지)
        if (x <= -10.0 or x >= 310.0 or z <= -10.0 or z >= 310.0): # Config 기준에 맞춰 수정 필요
            return float("inf")

        # 2. 전역 장애물과의 최소 거리 계산
        min_dist = float('inf')
        for obs_x, obs_z in state_manager.global_obstacles:
            dist = math.hypot(x - obs_x, z - obs_z)
            if dist < min_dist:
                min_dist = dist
        
        # 3. 거리에 따른 비용 부과
        if min_dist <= critical_dist:
            return float("inf")  # 충돌 위험 지역
        
        if min_dist < safe_dist:
            # 거리에 반비례하여 비용 증가 (지수적 패널티)
            # 1.0m(inf) ~ 2.5m(0) 사이의 비용 계산
            normalized_dist = (min_dist - critical_dist) / (safe_dist - critical_dist)
            total_cost += (1.0 / (normalized_dist + 0.1)) * 10.0

    return total_cost

class HybridController:
    """
    개선된 하이브리드 제어기 - DWA 우선 정책
    
    개선사항:
    1. DWA 우선 사용 - 유효 경로 없어도 최선의 DWA 선택
    2. PID 폴백 조건 완화 - 극단적 상황에서만 PID 사용
    3. 적응적 파라미터 - 장애물 밀도에 따라 조정
    """
    
    def __init__(self, config, planner, state_manager):
        self.config = config
        self.planner = planner
        self.state = state_manager
        
        self.dwa_config = ImprovedDWAConfig(config)
        self.steering_pid = PIDController(
            kp=config.PID.KP, 
            ki=config.PID.KI, 
            kd=config.PID.KD
        )
        
        self.last_velocity = 0.0
        self.last_yaw_rate = 0.0
        self.stuck_counter = 0
        self.last_position = None
        
        # 충돌/Stuck 복구 상태
        self.recovery_mode = False
        self.recovery_start_time = 0
        self.recovery_direction = 1
        
        # SEQ 4 재계획 관련
        self.last_replan_time = 0.0
        self.last_replan_distance = float('inf')
        
        # 🆕 DWA 연속 실패 카운터
        self.dwa_fail_count = 0
        self.max_dwa_fail_before_recovery = 5  # 5회 연속 실패 시 복구 모드
        
        # 🆕 DWA 통계
        self.dwa_success_count = 0
        self.dwa_total_count = 0
        
    def reset(self):
        """제어기 상태 초기화"""
        self.steering_pid.reset()
        self.last_velocity = 0.0
        self.last_yaw_rate = 0.0
        self.stuck_counter = 0
        self.last_position = None
        self.recovery_mode = False
        self.last_replan_time = 0.0
        self.last_replan_distance = float('inf')
        self.dwa_fail_count = 0
        self.dwa_success_count = 0
        self.dwa_total_count = 0

    def compute_action(self, curr_x, curr_z, curr_yaw):
        
        # 1. 위치 업데이트
        curr_x, curr_z = self.config.clamp_world_xz(curr_x, curr_z)
        self.state.update_robot_pose(curr_x, curr_z)

        if self.state.seq in [1, 3, 4]:
            self._update_obstacle_margin_for_seq()
        
        # 디버깅
        if not hasattr(self, '_compute_count'):
            self._compute_count = 0
        self._compute_count += 1
        if self._compute_count % 50 == 1:
            print(f"🚗 [compute_action] #{self._compute_count} SEQ={self.state.seq} pos=({curr_x:.1f},{curr_z:.1f}) dest={self.state.destination}")
        
        # 2. SEQ 2 사격 처리
        if self.state.seq == 2:
            cmd = self._stop_command()
            cmd["fire"] = True
            self.state.seq = 3
            self.state.status_message = "🔥 사격 완료! 다음 목적지를 선택하세요 (SEQ 3)"
            return cmd

        # 3. 목적지 없으면 정지
        if self.state.destination is None:
            return self._stop_command()
        
        # 4. 도착 확인 및 SEQ 전환
        dist_to_goal = math.hypot(
            self.state.destination[0] - curr_x, 
            self.state.destination[1] - curr_z
        )
        
        if dist_to_goal < self.config.ARRIVAL_THRESHOLD:
            print(f"✅ 도착! 거리={dist_to_goal:.2f}m (임계값={self.config.ARRIVAL_THRESHOLD}m)")
            
            if self.state.seq == 1:
                self.state.seq = 2
                self.state.status_message = "🎯 SEQ 1 도착! 사격 시스템 가동 중..."
                self.state.clear_path()
                self.state.destination = None
                print("🔄 SEQ 1→2 전환, 목적지 클리어")
                return self._stop_command()
                
            elif self.state.seq == 3:
                self.state.seq = 4
                self.state.status_message = "🚀 SEQ 3 도착! 최종 자율주행 모드 활성화 (A* + DWA)"
                self.state.clear_path()
                self.state.destination = None
                self.state.clear_global_obstacles()
                self.last_replan_time = 0.0
                self.last_replan_distance = float('inf')
                self.dwa_fail_count = 0  # DWA 실패 카운터 초기화
                print("🔄 SEQ 3→4 전환, A* + DWA 모드 시작")
                return self._stop_command()
                
            elif self.state.seq == 4:
                self.state.status_message = "🏁 모든 임무 완료!"
                self.state.clear_path()
                self.state.destination = None
                print(f"🏁 SEQ 4 완료! DWA 성공률: {self.dwa_success_count}/{self.dwa_total_count}")
                return self._stop_command()
            
            else:
                self.state.clear_path()
                self.state.destination = None
                print(f"⚠️ SEQ {self.state.seq} 도착 처리")
                return self._stop_command()

        # 5. Stuck 감지
        self._detect_stuck(curr_x, curr_z)
        
        # 6. Stuck 복구 모드 처리
        if self.stuck_counter >= self.config.Stuck.STUCK_COUNT_LIMIT:
            return self._recovery_action(curr_x, curr_z, curr_yaw)
        
        # 🆕 7. DWA 연속 실패 시 짧은 복구
        if self.dwa_fail_count >= self.max_dwa_fail_before_recovery:
            print(f"⚠️ DWA {self.dwa_fail_count}회 연속 실패 → 짧은 복구 시도")
            self.dwa_fail_count = 0
            return self._short_recovery_action(curr_x, curr_z, curr_yaw)
        
        # 8. SEQ에 따른 경로 생성 및 제어
        if self.state.seq == 4:
            return self._seq4_control(curr_x, curr_z, curr_yaw)
        else:
            return self._seq13_control(curr_x, curr_z, curr_yaw)
    
    def _update_obstacle_margin_for_seq(self):
        """현재 SEQ에 맞는 obstacle_margin 적용"""
        if self.state.seq == 4:
            new_margin = self.config.ASTAR.OBSTACLE_MARGIN_SEQ4
        else:
            new_margin = self.config.ASTAR.get_obstacle_margin(self.state.seq)
        
        if new_margin != self.planner.obstacle_margin:
            self.planner.set_obstacle_margin(new_margin)

    # ========== SEQ 4 제어 ==========
    
    def _seq4_control(self, curr_x, curr_z, curr_yaw):
        """SEQ 4: A* + DWA 제어 (개선된 버전)"""
        
        # 1. 초기 경로 생성
        if not self.state.global_path:
            self._generate_initial_path_seq4(curr_x, curr_z)
            if not self.state.global_path:
                return self._stop_command()
        
        # 3. 경로 업데이트
        self._update_path(curr_x, curr_z)
        
        if not self.state.global_path:
            return self._stop_command()
        
        # 4. 타겟 포인트 선택
        target_point, _ = self._select_target_point(curr_x, curr_z)
        if not target_point:
            return self._stop_command()
        
        # 5. 🆕 DWA 우선 제어 (개선된 버전)
        result = self._improved_dwa_control(curr_x, curr_z, curr_yaw, target_point)
        if result is None:
            result = self._pid_control(curr_x, curr_z, curr_yaw, target_point)
        
        # 6. 🆕 주기적으로 경로 + DWA trajectory 이미지 저장 (20회마다)
        if self._compute_count % 20 == 0:
            try:
                save_path_image(
                    self.planner,
                    self.state.global_path,
                    (curr_x, curr_z),
                    curr_yaw,
                    filename="SEQ4-path-debug.png",
                    title=f"SEQ4 Path + DWA (#{self._compute_count})",
                    state_manager=self.state
                )
            except Exception as e:
                pass
        
        return result
    
    def _generate_initial_path_seq4(self, curr_x, curr_z):
        """SEQ 4 초기 경로: 장애물 없이 순수 A* 경로"""
        dest = self.state.destination
        if dest is None:
            return
        
        print(f"🗺️ [SEQ 4] 초기 경로 생성 (장애물 무시)")
        print(f"   현재 위치: ({curr_x:.1f}, {curr_z:.1f})")
        print(f"   목표 위치: ({dest[0]:.1f}, {dest[1]:.1f})")
        
        self.planner.set_obstacles([])

        # 🔧 SEQ4 전용 범위: X(0~95), Z(0~300)
        self.planner.update_grid_range(0.0, 95.0, 0.0, 300.0)
        
        path = self.planner.find_path(
            (curr_x, curr_z), 
            (dest[0], dest[1]), 
            use_obstacles=False
        )
        
        if path:
            self.state.global_path = path
            print(f"✅ [SEQ 4] 초기 경로 생성 완료: {len(path)}개 노드")
        else:
            print(f"⚠️ [SEQ 4] 초기 경로 생성 실패!")

        try:
            save_path_image(
                planner=self.planner,
                path=path,
                current_pos=(curr_x, curr_z),
                current_yaw=self.state.robot_yaw_deg,
                filename="SEQ4-path-debug.png",
                title=f"SEQ {self.state.seq} - UPDATE",
                state_manager=self.state
            )
        except Exception as e:
            print(f"⚠️ 디버그 이미지 저장 실패: {e}")
    
    def _should_replan_seq4(self, curr_x, curr_z) -> bool:
        # """경로 재계획 필요 여부 판단"""
        # cfg = self.config.SEQ4
        
        # if len(self.state.global_path) < cfg.MIN_PATH_NODES:
        #     print(f"🔄 [SEQ 4] 경로 노드 부족({len(self.state.global_path)}개) → 재계획 필요")
        #     return True
        
        # if cfg.REPLAN_MODE.lower() == "distance":
        #     current_dist = self._distance_to_goal(curr_x, curr_z)
        #     distance_traveled = self.last_replan_distance - current_dist
            
        #     if distance_traveled >= cfg.REPLAN_DISTANCE_INTERVAL:
        #         print(f"🔄 [SEQ 4] {distance_traveled:.1f}m 진행 → 재계획")
        #         return True
                
        # elif cfg.REPLAN_MODE.lower() == "time":
        #     elapsed = time.time() - self.last_replan_time
            
        #     if elapsed >= cfg.REPLAN_TIME_INTERVAL:
        #         print(f"🔄 [SEQ 4] {elapsed:.1f}초 경과 → 재계획")
        #         return True
        
        return False
    
    def _replan_with_obstacles_seq4(self, curr_x, curr_z):
        """축적된 장애물로 A* 재계획"""
        dest = self.state.destination
        if not dest: return
        
        # 🔧 SEQ4 전용 범위: X(0~95), Z(0~300)
        self.planner.update_grid_range(0.0, 95.0, 0.0, 300.0)
                
        path = self.planner.find_path(
            (curr_x, curr_z), 
            (dest[0], dest[1]), 
            use_obstacles=True
        )
        
        if path and len(path) >= self.config.SEQ4.MIN_PATH_NODES:
            self.state.global_path = path
            self.state.global_path_version += 1
            self.last_replan_distance = self._distance_to_goal(curr_x, curr_z)
            self.last_replan_time = time.time()
            print(f"✅ [SEQ 4] 경로 재계획 완료: {len(path)}개 노드")
        else:
            print(f"⚠️ [SEQ 4] 장애물 반영 재계획 실패 → 장애물 무시 경로 생성")
            self.planner.set_obstacles([])
            path = self.planner.find_path(
                (curr_x, curr_z), 
                (dest[0], dest[1]), 
                use_obstacles=False
            )
            if path:
                self.state.global_path = path
                self.state.global_path_version += 1
                self.last_replan_distance = self._distance_to_goal(curr_x, curr_z)
                self.last_replan_time = time.time()
                print(f"✅ [SEQ 4] 장애물 무시 경로 생성: {len(path)}개 노드")
        
        try:
            save_path_image(
                self.planner,
                self.state.global_path,
                (curr_x, curr_z),
                self.state.robot_yaw_deg,
                filename="SEQ4-path-debug.png",
                title="SEQ 4 Autonomous Path",
                state_manager=self.state
            )
        except:
            pass
    
    def _update_astar_with_global_obstacles(self, curr_x, curr_z):
        """전역 장애물을 A* 플래너에 반영"""
        obstacles = []
        grid_size = self.state.global_obstacle_grid_size
        clear_radius = self.config.SEQ4.ROBOT_CLEAR_RADIUS
        
        for (x, z) in self.state.global_obstacles:
            dist = math.hypot(x - curr_x, z - curr_z)
            if dist < clear_radius:
                continue
            
            obs = ObstacleRect.from_min_max(
                x_min=x, x_max=x + grid_size,
                z_min=z, z_max=z + grid_size
            )
            obstacles.append(obs)
        
        self.planner.set_obstacles(obstacles)
        print(f"📍 A*에 전역 장애물 반영: {len(obstacles)}개 (전차 주변 {clear_radius}m 제외)")
    
    def _distance_to_goal(self, curr_x, curr_z) -> float:
        """현재 위치에서 목적지까지 거리"""
        if self.state.destination is None:
            return float('inf')
        return math.hypot(
            self.state.destination[0] - curr_x,
            self.state.destination[1] - curr_z
        )
    
    def _check_path_blocked(self) -> bool:
        """현재 경로가 장애물로 막혔는지 확인"""
        if not self.state.global_path or self.state.costmap is None:
            return False
        
        costmap = self.state.costmap
        origin = self.state.costmap_origin
        grid_size = self.config.Lidar.GRID_SIZE
        
        check_count = min(10, len(self.state.global_path))
        
        for i in range(check_count):
            px, pz = self.state.global_path[i]
            
            cm_x = int((px - origin[0]) / grid_size)
            cm_z = int((pz - origin[1]) / grid_size)
            
            if 0 <= cm_x < costmap.shape[1] and 0 <= cm_z < costmap.shape[0]:
                if costmap[cm_z, cm_x] >= 0.7:
                    return True
        
        return False

    # ========== SEQ 1, 3 제어 ==========
    
    def _seq13_control(self, curr_x, curr_z, curr_yaw):
        """SEQ 1, 3: A* + PID 제어"""
        
        if not self.state.global_path:
            self._generate_path(curr_x, curr_z)
            if not self.state.global_path:
                return self._stop_command()
        
        self._update_path_seq13(curr_x, curr_z)
        
        target_point, _ = self._select_target_point(curr_x, curr_z)
        if not target_point:
            return self._stop_command()

        return self._pid_control(curr_x, curr_z, curr_yaw, target_point)
    
    def _generate_path(self, curr_x, curr_z):
        """SEQ별 범위 설정 후 A* 경로 생성"""

        mask_zones = []
        use_obstacles = True
        
        if self.state.seq == 1:
            forbidden_zone = ObstacleRect.from_min_max(158.0, 190.0, 115.0, 156.0)
            mask_zones.append(forbidden_zone)
            self.planner.update_grid_range(0.0, 300.0, 0.0, 300.0)

        elif self.state.seq == 3:
            self.planner.update_grid_range(0.0, 300.0, 0.0, 300.0)
        
        dest = self.state.destination
        if dest is None:
            return
        
        self.planner.set_mask_zones(mask_zones)

        path = self.planner.find_path(
            (curr_x, curr_z),
            (dest[0], dest[1]),
            use_obstacles=use_obstacles
        )
        
        if path:
            self.state.global_path = path
            self.state.global_path_version += 1
            # 🆕 이미지 저장 추가
            try:
                obs_count = len(self.planner._obstacles) if self.planner._obstacles else 0
                mode_label = f"A* + PID (SEQ {self.state.seq})"
                
                save_path_image(
                    planner=self.planner,
                    path=path,
                    current_pos=(curr_x, curr_z),
                    current_yaw=self.state.robot_yaw_deg,
                    filename=f"SEQ {self.state.seq}_path_debug.png",
                    title=f"SEQ {self.state.seq} - {mode_label}",
                    state_manager=self.state
                )
                print(f"💾 경로 이미지 저장 완료: path_debug.png ({len(path)}개 노드, 장애물 {obs_count}개)")
            except Exception as e:
                print(f"⚠️ 디버그 이미지 저장 실패: {e}")

    # ========== 경로 추종 ==========
    
    def _update_path_seq13(self, curr_x, curr_z):
        """SEQ 1, 3: 가장 가까운 포인트 기반 업데이트"""
        if not self.state.global_path:
            return
        
        # 1. 전체 경로에서 가장 가까운 포인트 찾기
        min_dist = float('inf')
        closest_idx = 0
        
        for i in range(len(self.state.global_path)):
            d = math.hypot(
                self.state.global_path[i][0] - curr_x, 
                self.state.global_path[i][1] - curr_z
            )
            if d < min_dist:
                min_dist = d
                closest_idx = i
        
        # 2. 그 이전 포인트들 모두 제거
        if closest_idx > 0 and len(self.state.global_path) > 5:
            removed = closest_idx
            self.state.global_path = self.state.global_path[closest_idx:]
            print(f"🔄 [SEQ {self.state.seq}] 경로 업데이트: {removed}개 제거, 남은: {len(self.state.global_path)}")

    def _update_path(self, curr_x, curr_z):
        """경로 업데이트 (지나간 웨이포인트 제거)"""
        if not self.state.global_path:
            return
        
        removed_count = 0
        
        while len(self.state.global_path) > 1:
            next_wp = self.state.global_path[0]
            dist = math.hypot(next_wp[0] - curr_x, next_wp[1] - curr_z)
            
            if dist < self.config.MIN_TARGET_DIST:
                self.state.global_path.pop(0)
                removed_count += 1
            else:
                break
        
        if removed_count > 0:
            print(f"🔄 [SEQ 4] 경로 업데이트: {removed_count}개 제거, 남은: {len(self.state.global_path)}")
    
    def _select_target_point(self, curr_x, curr_z):
        """타겟 포인트 선택 (Lookahead)"""
        if not self.state.global_path:
            return None, 0
        
        lookahead = self.config.LOOKAHEAD_DIST
        
        # Lookahead 거리에 맞는 포인트 찾기
        cumulative_dist = 0.0
        prev_point = (curr_x, curr_z)
        
        for i, point in enumerate(self.state.global_path):
            segment_dist = math.hypot(
                point[0] - prev_point[0],
                point[1] - prev_point[1]
            )
            cumulative_dist += segment_dist
            
            if cumulative_dist >= lookahead:
                return point, i
            
            prev_point = point
        
        # 경로 끝까지 도달하면 마지막 포인트 반환
        return self.state.global_path[-1], len(self.state.global_path) - 1

    # ========== Stuck 감지/복구 ==========
    
    def _detect_stuck(self, curr_x, curr_z):
        """Stuck 감지"""
        if self.last_position is None:
            self.last_position = (curr_x, curr_z)
            return
        
        dist = math.hypot(
            curr_x - self.last_position[0],
            curr_z - self.last_position[1]
        )
        
        if dist < self.config.Stuck.STUCK_THRESHOLD:
            self.stuck_counter += 1
        else:
            if self.stuck_counter > 0:
                print(f"✅ 탈출 성공! stuck_counter={self.stuck_counter} → 0")
            self.stuck_counter = 0
        
        self.last_position = (curr_x, curr_z)
    
    def _recovery_action(self, curr_x, curr_z, curr_yaw):
        """Stuck 복구 동작 (후진 + 회전)"""
        rc = self.config.Recovery
        
        if not self.recovery_mode:
            self.recovery_mode = True
            self.recovery_start_time = time.time()
            self.recovery_direction = 1 if (self.stuck_counter % 2 == 0) else -1
            print(f"🔧 복구 시작: {'좌회전' if self.recovery_direction > 0 else '우회전'} 후진")
        
        elapsed = time.time() - self.recovery_start_time
        
        if elapsed < rc.PHASE1_SEC:
            return {
                "moveWS": {"command": "S", "weight": rc.PHASE1_WS_WEIGHT},
                "moveAD": {"command": "D" if self.recovery_direction > 0 else "A", "weight": rc.PHASE1_AD_WEIGHT},
                "fire": False
            }
        
        elif elapsed < rc.PHASE1_SEC + rc.PHASE2_SEC:
            return {
                "moveWS": {"command": "STOP", "weight": 1.0},
                "moveAD": {"command": "D" if self.recovery_direction > 0 else "A", "weight": rc.PHASE2_AD_WEIGHT},
                "fire": False
            }
        
        else:
            print("✅ 복구 완료! 경로 재생성...")
            self.recovery_mode = False
            self.stuck_counter = 0
            self.last_position = None
            self.state.clear_path()
            self.dwa_fail_count = 0  # DWA 실패 카운터도 초기화
            
            return self._stop_command()
    
    def _short_recovery_action(self, curr_x, curr_z, curr_yaw):
        """🆕 DWA 연속 실패 시 짧은 복구 (후진 1초)"""
        return {
            "moveWS": {"command": "S", "weight": 0.5},
            "moveAD": {"command": "D" if (self._compute_count % 2 == 0) else "A", "weight": 0.3},
            "fire": False
        }

    # ========== 개선된 DWA 제어 ==========
    
    def _improved_dwa_control(self, curr_x, curr_z, curr_yaw, target_node):

        # 가상 라이다 디버깅
        if self.state.global_obstacles:
            nearby_obs = [
                obs for obs in self.state.global_obstacles
                if math.hypot(curr_x - obs[0], curr_z - obs[1]) < 10.0
            ]
            if self._compute_count % 20 == 0:
                print(f"[DEBUG] 가상 라이다 작동 중: 주변 10m내 장애물 {len(nearby_obs)}개 감지됨")

        self.dwa_total_count += 1
        
        curr_yaw_rad = math.radians(curr_yaw)
        x = np.array([curr_x, curr_z, curr_yaw_rad, self.last_velocity, self.last_yaw_rate])

        original_predict = self.dwa_config.predict_time
        adaptive_active = False
        
        # 🆕 적응적 예측 시간 (장애물 많으면 짧게)
        if self.dwa_config.adaptive_mode and self.state.costmap is not None:
            obstacle_ratio = np.sum(self.state.costmap >= 0.5) / max(self.state.costmap.size, 1)
            # 장애물 많으면 예측 시간 줄임
            adaptive_predict = self.dwa_config.max_predict_time - \
                              (obstacle_ratio * (self.dwa_config.max_predict_time - self.dwa_config.min_predict_time))
            original_predict = self.dwa_config.predict_time
            self.dwa_config.predict_time = max(self.dwa_config.min_predict_time, adaptive_predict)
            adaptive_active = True
        
        dw = calc_dynamic_window(x, self.dwa_config)
        
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])
        valid_trajectories = 0
        total_trajectories = 0
        
        for v in np.arange(dw[0], dw[1], self.dwa_config.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.dwa_config.yaw_rate_resolution):
                total_trajectories += 1
                trajectory = predict_trajectory(x, v, omega, self.dwa_config)
                
                to_goal_cost = self.dwa_config.to_goal_cost_gain * calc_to_goal_cost(trajectory, [target_node[0], target_node[1]])
                speed_cost = self.dwa_config.speed_cost_gain * (self.dwa_config.max_speed - trajectory[-1, 3])
                ob_cost = self.dwa_config.obstacle_cost_gain * calc_virtual_lidar_cost(trajectory, self.state, self.dwa_config)
                steering_penalty = abs(omega) * self.dwa_config.steering_penalty
                final_cost = to_goal_cost + speed_cost + ob_cost + steering_penalty
                
                if final_cost < float('inf'):
                    valid_trajectories += 1
                    if final_cost < min_cost:
                        min_cost = final_cost
                        best_u = [v, omega]
                        best_trajectory = trajectory
                        # 최선의 궤적 발견 시 비용 분석 저장
                        if self._compute_count % 20 == 0:
                            best_cost_breakdown = {
                                'to_goal': round(to_goal_cost, 2),
                                'speed': round(speed_cost, 2),
                                'obstacle': round(ob_cost, 2),
                                'steering': round(steering_penalty, 2)
                            }
        
        # 적응적 예측 시간 복원
        if adaptive_active:
            self.dwa_config.predict_time = original_predict
        
        self.state.valid_traj_count = valid_trajectories
        
        if self._compute_count % 20 == 0:
            breakdown_str = ""
            if 'best_cost_breakdown' in locals():
                breakdown_str = f" [goal:{best_cost_breakdown['to_goal']}, spd:{best_cost_breakdown['speed']}, obs:{best_cost_breakdown['obstacle']}, str:{best_cost_breakdown['steering']}]"
            print(f"🎯 DWA: 총={total_trajectories}, 유효={valid_trajectories}, "
                  f"비용={min_cost:.2f}{breakdown_str}, v={best_u[0]:.2f}, ω={best_u[1]:.3f}")
        
        # 🆕 개선된 폴백 로직 - DWA 실패 시 PID 사용
        if valid_trajectories == 0:
            self.dwa_fail_count += 1
            print(f"⚠️ DWA 유효 경로 없음 → PID 폴백 (fail #{self.dwa_fail_count})")
            return self._pid_control(curr_x, curr_z, curr_yaw, target_node)
        else:
            self.dwa_fail_count = 0  # 성공하면 카운터 초기화
            self.dwa_success_count += 1
        
        # Stuck 방지
        if (abs(best_u[0]) < self.dwa_config.robot_stuck_flag_cons and 
            abs(x[3]) < self.dwa_config.robot_stuck_flag_cons):
            best_u[0] = -0.1
            best_u[1] = 0.0
        
        self.state.last_dwa_traj = best_trajectory
        self.state.last_dwa_target = (float(target_node[0]), float(target_node[1]))
        self.state.local_traj_version += 1

        desired_v = float(best_u[0])
        desired_omega = float(best_u[1])

        if (abs(desired_v) < self.dwa_config.robot_stuck_flag_cons) and (abs(x[3]) < self.dwa_config.robot_stuck_flag_cons):
            desired_v = -float(self.config.Recovery.REVERSE_SPEED)
            desired_omega = 0.0

        self.last_velocity = desired_v
        self.last_yaw_rate = desired_omega

        steer_command = desired_omega / self.dwa_config.max_yaw_rate
        steer_command = max(min(steer_command, 1.0), -1.0)
        steer_weight = abs(steer_command)

        if abs(steer_command) < 0.05:
            steer_dir = ""
            steer_weight = 0.0
        else:
            steer_dir = "D" if steer_command > 0 else "A"
        
        ws_cmd = "W" if desired_v > 0.05 else ("S" if desired_v < -0.05 else "STOP")
        ws_weight = min(max(abs(desired_v) / self.dwa_config.max_speed, 0.0), 1.0)
        
        return {
            "moveWS": {"command": ws_cmd, "weight": round(ws_weight, 2)},
            "moveAD": {"command": steer_dir, "weight": round(steer_weight, 2)},
            "fire": False
        }
    
    def _pid_control(self, curr_x, curr_z, curr_yaw, target_node):
        """PID 제어"""
        dx = target_node[0] - curr_x
        dz = target_node[1] - curr_z
        target_angle_deg = math.degrees(math.atan2(dx, dz))
        
        error = target_angle_deg - curr_yaw

        while error > 180: 
            error -= 360
        while error < -180: 
            error += 360
        
        if self._compute_count % 20 == 0:
            print(f"🎯 PID: pos=({curr_x:.1f},{curr_z:.1f}) → target=({target_node[0]:.1f},{target_node[1]:.1f}), error={error:.1f}°")

        pid_output = self.steering_pid.compute(error)
        
        steer_weight = min(abs(pid_output), 1.0)
        steer_dir = "D" if pid_output > 0 else "A"
        if pid_output == 0: 
            steer_dir = ""
        
        max_w = self.config.PID.MAX_SPEED_WEIGHT
        min_w = self.config.PID.MIN_SPEED_WEIGHT
        gain = self.config.PID.SPEED_REDUCT_GAIN
        error_th = self.config.PID.ERROR_THRESHOLD
        error_range = self.config.PID.ERROR_RANGE

        speed_weight = max(min_w, max_w - steer_weight * gain)
        if abs(error) > error_th:
            reduction_factor = max(0.0, 1.0 - (abs(error) - error_th) / error_range)
            speed_weight *= reduction_factor
        speed_weight = max(speed_weight, min_w)
        
        if speed_weight <= 0.05:
            cmd_ws = "STOP"
            speed_weight = 1.0
        else:
            cmd_ws = "W"
        
        return {
            "moveWS": {"command": cmd_ws, "weight": round(speed_weight, 2)},
            "moveAD": {"command": steer_dir, "weight": round(steer_weight * self.config.PID.STEER_SENSITIVITY, 2)},
            "fire": False
        }
    
    @staticmethod
    def _stop_command():
        """정지 명령"""
        return {
            "moveWS": {"command": "STOP", "weight": 1.0},
            "moveAD": {"command": "", "weight": 0.0}, 
            "fire": False
        }