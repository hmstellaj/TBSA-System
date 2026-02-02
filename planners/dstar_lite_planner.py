"""
D* Lite 전역 경로 계획기 (Dynamic Replanning)

D* Lite 알고리즘:
- 목표에서 시작점 방향으로 탐색 (역방향)
- 환경이 변할 때 전체 경로를 재계산하지 않고 영향받는 부분만 업데이트
- LiDAR costmap 변화에 따른 실시간 경로 수정에 최적화

참고: Koenig & Likhachev (2002) "D* Lite"
"""

from __future__ import annotations

import math
import heapq
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass, field
import numpy as np

from config import Config


@dataclass
class DStarNode:
    """D* Lite 노드"""
    x: int
    z: int
    g: float = float('inf')  # 목표까지의 실제 비용
    rhs: float = float('inf')  # one-step lookahead 비용
    key: Tuple[float, float] = field(default_factory=lambda: (float('inf'), float('inf')))
    
    def __hash__(self):
        return hash((self.x, self.z))
    
    def __eq__(self, other):
        if isinstance(other, DStarNode):
            return self.x == other.x and self.z == other.z
        return False
    
    def __lt__(self, other):
        return self.key < other.key


class DStarLitePlanner:
    """
    D* Lite 기반 동적 경로 계획기
    
    특징:
    - 초기 경로 계획 후, costmap 변화 시 증분 업데이트
    - 전차의 이동 방향 (시작→목표)으로 경로 반환
    - LiDAR로 발견한 새 장애물을 실시간 반영
    """
    
    # 비용 상수
    COST_STRAIGHT = Config.ASTAR.COST_STRAIGHT
    COST_DIAGONAL = Config.ASTAR.COST_DIAGONAL
    INF = float('inf')
    
    # 장애물 관련
    OBSTACLE_THRESHOLD = 0.7  # costmap에서 장애물로 간주할 값
    LETHAL_COST = 1.0  # 절대 통과 불가
    
    def __init__(
        self,
        grid_min_x: float = 0.0,
        grid_max_x: float = 300.0,
        grid_min_z: float = 0.0,
        grid_max_z: float = 300.0,
        cell_size: float = 1.0,
        obstacle_margin: float = 3.0,
        allow_diagonal: bool = True,
        state_manager = None  # 🆕 전역 장애물 맵 접근용
    ):
        self.grid_min_x = float(grid_min_x)
        self.grid_max_x = float(grid_max_x)
        self.grid_min_z = float(grid_min_z)
        self.grid_max_z = float(grid_max_z)
        self.cell_size = float(cell_size)
        self.obstacle_margin = float(obstacle_margin)
        self.allow_diagonal = allow_diagonal
        self.state_manager = state_manager  # 🆕 전역 장애물 참조용
        
        # 그리드 크기 계산
        self.grid_size_x = max(1, int(math.ceil((self.grid_max_x - self.grid_min_x) / self.cell_size)))
        self.grid_size_z = max(1, int(math.ceil((self.grid_max_z - self.grid_min_z) / self.cell_size)))
        
        # D* Lite 상태
        self.nodes: Dict[Tuple[int, int], DStarNode] = {}
        self.open_list: List[DStarNode] = []
        self.k_m: float = 0.0  # 로봇 이동에 따른 휴리스틱 보정값
        
        # 시작/목표 노드
        self.start_node: Optional[DStarNode] = None
        self.goal_node: Optional[DStarNode] = None
        
        # Costmap 저장
        self.costmap: Optional[np.ndarray] = None
        self.costmap_origin: Optional[Tuple[float, float]] = None
        self.prev_costmap: Optional[np.ndarray] = None
        
        # 초기화 상태
        self.initialized = False
        self.last_robot_pos: Optional[Tuple[int, int]] = None
        
        # 마스킹 영역 (No-Go Zone)
        self._mask_zones: List = []
        
    def update_grid_range(self, min_x: float, max_x: float, min_z: float, max_z: float):
        """그리드 범위 동적 업데이트"""
        self.grid_min_x = float(min_x)
        self.grid_max_x = float(max_x)
        self.grid_min_z = float(min_z)
        self.grid_max_z = float(max_z)
        
        self.grid_size_x = max(1, int(math.ceil((self.grid_max_x - self.grid_min_x) / self.cell_size)))
        self.grid_size_z = max(1, int(math.ceil((self.grid_max_z - self.grid_min_z) / self.cell_size)))
        
        # 그리드 범위가 바뀌면 재초기화 필요
        self.initialized = False
        self.nodes.clear()
        self.open_list.clear()
        
        print(f"📏 D* Lite 범위 변경: X({min_x}~{max_x}), Z({min_z}~{max_z})")
    
    def set_mask_zones(self, zones: List):
        """마스킹 영역 설정"""
        self._mask_zones = zones
        self.initialized = False  # 재초기화 필요
        print(f"🚫 D* Lite 마스킹 영역 {len(zones)}개 설정")
    
    def _get_node(self, x: int, z: int) -> DStarNode:
        """노드 가져오기 (없으면 생성)"""
        key = (x, z)
        if key not in self.nodes:
            self.nodes[key] = DStarNode(x=x, z=z)
        return self.nodes[key]
    
    def _world_to_grid(self, world_x: float, world_z: float) -> Tuple[int, int]:
        """월드 좌표 → 그리드 인덱스"""
        gx = int((world_x - self.grid_min_x) / self.cell_size)
        gz = int((world_z - self.grid_min_z) / self.cell_size)
        gx = max(0, min(gx, self.grid_size_x - 1))
        gz = max(0, min(gz, self.grid_size_z - 1))
        return gx, gz
    
    def _grid_to_world(self, gx: int, gz: int) -> Tuple[float, float]:
        """그리드 인덱스 → 월드 좌표 (셀 중심)"""
        world_x = self.grid_min_x + (gx + 0.5) * self.cell_size
        world_z = self.grid_min_z + (gz + 0.5) * self.cell_size
        return world_x, world_z
    
    def _heuristic(self, node: DStarNode, target: DStarNode) -> float:
        """휴리스틱 (Chebyshev distance for 8-directional movement)"""
        dx = abs(node.x - target.x)
        dz = abs(node.z - target.z)
        
        if self.allow_diagonal:
            # Chebyshev distance with diagonal cost
            diag = min(dx, dz)
            straight = abs(dx - dz)
            return self.COST_DIAGONAL * diag + self.COST_STRAIGHT * straight
        else:
            return self.COST_STRAIGHT * (dx + dz)
    
    def _calculate_key(self, node: DStarNode) -> Tuple[float, float]:
        """D* Lite 키 계산"""
        min_g_rhs = min(node.g, node.rhs)
        if self.start_node:
            h = self._heuristic(self.start_node, node)
        else:
            h = 0
        return (min_g_rhs + h + self.k_m, min_g_rhs)
    
    def _update_node_key(self, node: DStarNode):
        """노드의 키 업데이트"""
        node.key = self._calculate_key(node)
    
    def _get_cost(self, from_node: DStarNode, to_node: DStarNode) -> float:
        """두 노드 사이의 이동 비용 계산"""
        # 범위 체크
        if not (0 <= to_node.x < self.grid_size_x and 0 <= to_node.z < self.grid_size_z):
            return self.INF
        
        # 월드 좌표 계산
        to_world_x, to_world_z = self._grid_to_world(to_node.x, to_node.z)
        
        # 마스킹 영역 체크
        for zone in self._mask_zones:
            if (zone.x_min <= to_world_x <= zone.x_max and 
                zone.z_min <= to_world_z <= zone.z_max):
                return self.INF
        
        # Costmap 기반 비용 계산
        costmap_cost = self._get_costmap_value(to_node.x, to_node.z)
        
        if costmap_cost >= self.LETHAL_COST:
            return self.INF
        
        # 기본 이동 비용 계산
        dx = abs(from_node.x - to_node.x)
        dz = abs(from_node.z - to_node.z)
        
        if dx + dz == 2:  # 대각선 이동
            base_cost = self.COST_DIAGONAL
        else:
            base_cost = self.COST_STRAIGHT
        
        # costmap 비용을 가중치로 적용
        # costmap_cost가 높을수록 더 비용이 증가
        weighted_cost = base_cost * (1.0 + costmap_cost * 5.0)
        
        return weighted_cost
    
    def _get_costmap_value(self, gx: int, gz: int) -> float:
        """특정 그리드 셀의 costmap 값 반환 (전역 장애물 포함)"""
        # 월드 좌표 계산
        world_x, world_z = self._grid_to_world(gx, gz)
        
        # 🆕 전역 장애물 맵 체크 (먼저 확인)
        if self.state_manager is not None:
            if self.state_manager.is_global_obstacle(world_x, world_z):
                return self.LETHAL_COST  # 전역 장애물이면 통과 불가
        
        # 기존 Costmap 체크
        if self.costmap is None or self.costmap_origin is None:
            return 0.0  # costmap이 없으면 free로 간주
        
        # Costmap 인덱스 계산
        cm_x = int((world_x - self.costmap_origin[0]) / self.cell_size)
        cm_z = int((world_z - self.costmap_origin[1]) / self.cell_size)
        
        # 범위 체크
        if cm_x < 0 or cm_z < 0 or cm_z >= self.costmap.shape[0] or cm_x >= self.costmap.shape[1]:
            return 0.0  # costmap 범위 밖은 unknown = free로 간주
        
        return float(self.costmap[cm_z, cm_x])
    
    def _get_neighbors(self, node: DStarNode) -> List[DStarNode]:
        """이웃 노드 반환"""
        neighbors = []
        
        for dx in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dz == 0:
                    continue
                
                # 대각선 이동 허용 여부
                if not self.allow_diagonal and abs(dx) + abs(dz) > 1:
                    continue
                
                nx, nz = node.x + dx, node.z + dz
                
                if 0 <= nx < self.grid_size_x and 0 <= nz < self.grid_size_z:
                    neighbors.append(self._get_node(nx, nz))
        
        return neighbors
    
    def _update_vertex(self, node: DStarNode):
        """노드 업데이트 (D* Lite 핵심)"""
        if node != self.goal_node:
            # rhs = min over successors (c(s,s') + g(s'))
            min_rhs = self.INF
            for succ in self._get_neighbors(node):
                cost = self._get_cost(node, succ)
                if cost < self.INF:
                    new_rhs = cost + succ.g
                    if new_rhs < min_rhs:
                        min_rhs = new_rhs
            node.rhs = min_rhs
        
        # Open list에서 제거 (있으면)
        if node in self.open_list:
            self.open_list.remove(node)
            heapq.heapify(self.open_list)
        
        # g != rhs면 open list에 추가
        if node.g != node.rhs:
            self._update_node_key(node)
            heapq.heappush(self.open_list, node)
    
    def _compute_shortest_path(self):
        """최단 경로 계산 (D* Lite main loop)"""
        iterations = 0
        max_iterations = 50000  # 최대 반복 제한 (300x300 그리드에서 합리적인 값)
        
        import time
        start_time = time.time()
        timeout = 5.0  # 5초 타임아웃
        
        while self.open_list:
            # 타임아웃 체크
            if time.time() - start_time > timeout:
                print(f"⚠️ D* Lite: 타임아웃 ({timeout}초), {iterations}회 반복 후 중단")
                break
            
            if iterations > max_iterations:
                print(f"⚠️ D* Lite: 최대 반복 횟수 초과 ({max_iterations})")
                break
            
            iterations += 1
            
            # 진행 로그 (5000회마다)
            if iterations % 5000 == 0:
                elapsed = time.time() - start_time
                print(f"   D* Lite 진행 중: {iterations}회 반복, {elapsed:.2f}초 경과")
            
            # 시작 노드의 키 계산
            self._update_node_key(self.start_node)
            
            # 종료 조건 체크
            top_node = self.open_list[0]
            if top_node.key >= self.start_node.key and self.start_node.rhs == self.start_node.g:
                break
            
            # Pop minimum key node
            u = heapq.heappop(self.open_list)
            k_old = u.key
            self._update_node_key(u)
            
            if k_old < u.key:
                # 키가 업데이트됨, 다시 삽입
                heapq.heappush(self.open_list, u)
            elif u.g > u.rhs:
                # Overconsistent: g를 낮춤
                u.g = u.rhs
                for pred in self._get_neighbors(u):
                    self._update_vertex(pred)
            else:
                # Underconsistent: g를 infinity로
                u.g = self.INF
                self._update_vertex(u)
                for pred in self._get_neighbors(u):
                    self._update_vertex(pred)
        
        elapsed = time.time() - start_time
        if iterations > 0:
            print(f"🔍 D* Lite: {iterations}회 반복, {elapsed:.2f}초 소요")
    
    def initialize(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """D* Lite 초기화"""
        # 그리드 인덱스 변환
        sx, sz = self._world_to_grid(start[0], start[1])
        gx, gz = self._world_to_grid(goal[0], goal[1])
        
        # 노드 초기화
        self.nodes.clear()
        self.open_list.clear()
        self.k_m = 0.0
        
        # 시작/목표 노드 설정
        self.start_node = self._get_node(sx, sz)
        self.goal_node = self._get_node(gx, gz)
        
        # 목표 노드 초기화
        self.goal_node.rhs = 0.0
        self._update_node_key(self.goal_node)
        heapq.heappush(self.open_list, self.goal_node)
        
        # 초기 경로 계산
        self._compute_shortest_path()
        
        self.initialized = True
        self.last_robot_pos = (sx, sz)
        
        print(f"✅ D* Lite 초기화 완료: 시작({sx},{sz}) → 목표({gx},{gz})")
    
    def update_costmap(self, costmap: np.ndarray, origin: Tuple[float, float]):
        """Costmap 업데이트 및 경로 수정"""
        self.prev_costmap = self.costmap
        self.costmap = costmap
        self.costmap_origin = origin
        
        if not self.initialized or self.start_node is None:
            return
        
        # 변경된 셀 찾기
        changed_cells = self._find_changed_cells()
        
        if not changed_cells:
            return
        
        MAX_CHANGED_CELLS = 200
        if len(changed_cells) > MAX_CHANGED_CELLS:
            print(f"⚠️ D* Lite: 변경 셀 {len(changed_cells)}개 → 재초기화 필요")
            self.initialized = False  # ← 스킵 대신 재초기화 플래그
            return
        
        print(f"🔄 D* Lite: {len(changed_cells)}개 셀 변경 감지, 경로 업데이트 중...")
        
        # 변경된 셀 주변의 노드 업데이트
        for gx, gz in changed_cells:
            node = self._get_node(gx, gz)
            self._update_vertex(node)
            
            # 이웃 노드도 업데이트
            for neighbor in self._get_neighbors(node):
                self._update_vertex(neighbor)
        
        # 경로 재계산
        self._compute_shortest_path()
    
    def _find_changed_cells(self) -> Set[Tuple[int, int]]:
        """이전 costmap과 비교하여 변경된 셀 찾기"""
        changed = set()
        
        if self.prev_costmap is None or self.costmap is None:
            return changed
        
        if self.costmap_origin is None:
            return changed
        
        # 전체 그리드 순회 (최적화 필요 시 costmap 범위만)
        for gx in range(self.grid_size_x):
            for gz in range(self.grid_size_z):
                old_cost = self._get_prev_costmap_value(gx, gz)
                new_cost = self._get_costmap_value(gx, gz)
                
                # 장애물 상태 변화 감지 (threshold 기반)
                old_blocked = old_cost >= self.OBSTACLE_THRESHOLD
                new_blocked = new_cost >= self.OBSTACLE_THRESHOLD
                
                if old_blocked != new_blocked:
                    changed.add((gx, gz))
        
        return changed
    
    def _get_prev_costmap_value(self, gx: int, gz: int) -> float:
        """이전 costmap 값 반환"""
        if self.prev_costmap is None or self.costmap_origin is None:
            return 0.0
        
        world_x, world_z = self._grid_to_world(gx, gz)
        
        cm_x = int((world_x - self.costmap_origin[0]) / self.cell_size)
        cm_z = int((world_z - self.costmap_origin[1]) / self.cell_size)
        
        if cm_x < 0 or cm_z < 0 or cm_z >= self.prev_costmap.shape[0] or cm_x >= self.prev_costmap.shape[1]:
            return 0.0
        
        return float(self.prev_costmap[cm_z, cm_x])
    
    def update_robot_position(self, new_pos: Tuple[float, float]):
        """로봇 위치 업데이트 (k_m 보정)"""
        if not self.initialized or self.last_robot_pos is None:
            return
        
        new_grid = self._world_to_grid(new_pos[0], new_pos[1])
        
        if new_grid != self.last_robot_pos:
            # k_m 업데이트 (휴리스틱 보정)
            old_node = self._get_node(self.last_robot_pos[0], self.last_robot_pos[1])
            new_node = self._get_node(new_grid[0], new_grid[1])
            self.k_m += self._heuristic(old_node, new_node)
            
            # 시작 노드 업데이트
            self.start_node = new_node
            self.last_robot_pos = new_grid
    
    def get_path(self) -> List[Tuple[float, float]]:
        """현재 계산된 경로 반환 (월드 좌표)"""
        if not self.initialized or self.start_node is None or self.goal_node is None:
            return []
        
        path = []
        current = self.start_node
        visited = set()
        max_steps = self.grid_size_x * self.grid_size_z
        
        while current != self.goal_node and len(visited) < max_steps:
            if (current.x, current.z) in visited:
                print("⚠️ D* Lite: 경로에서 사이클 감지!")
                break
            
            visited.add((current.x, current.z))
            path.append(self._grid_to_world(current.x, current.z))
            
            # 다음 노드 선택 (가장 낮은 g 값을 가진 이웃)
            best_next = None
            best_cost = self.INF
            
            for neighbor in self._get_neighbors(current):
                cost = self._get_cost(current, neighbor) + neighbor.g
                if cost < best_cost:
                    best_cost = cost
                    best_next = neighbor
            
            if best_next is None:
                print("⚠️ D* Lite: 다음 노드를 찾을 수 없음!")
                break
            
            current = best_next
        
        if current == self.goal_node:
            path.append(self._grid_to_world(self.goal_node.x, self.goal_node.z))
        
        return path
    
    def find_path(
        self, 
        start: Tuple[float, float], 
        goal: Tuple[float, float],
        costmap: Optional[np.ndarray] = None,
        costmap_origin: Optional[Tuple[float, float]] = None,
        use_obstacles: bool = True  # A* 호환성을 위한 더미 파라미터
    ) -> List[Tuple[float, float]]:
        """
        경로 찾기 (A* 인터페이스 호환)
        
        Args:
            start: 시작점 (world_x, world_z)
            goal: 목표점 (world_x, world_z)
            costmap: LiDAR costmap (optional)
            costmap_origin: costmap 원점 (optional)
            use_obstacles: 장애물 사용 여부 (A* 호환용)
        
        Returns:
            경로 리스트 [(x, z), ...]
        """
        # Costmap 업데이트
        if costmap is not None and costmap_origin is not None:
            self.costmap = costmap
            self.costmap_origin = costmap_origin
        
        # 초기화 또는 목표 변경 시 재초기화
        if not self.initialized:
            self.initialize(start, goal)
        else:
            # 목표가 변경되었는지 확인
            gx, gz = self._world_to_grid(goal[0], goal[1])
            if self.goal_node is None or (gx, gz) != (self.goal_node.x, self.goal_node.z):
                self.initialize(start, goal)
            else:
                # 로봇 위치만 업데이트
                self.update_robot_position(start)
        
        return self.get_path()
    
    def replan(self, current_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """현재 위치에서 재계획"""
        if not self.initialized or self.goal_node is None:
            return []
        
        self.update_robot_position(current_pos)
        self._compute_shortest_path()
        return self.get_path()
    
    # ============================================================
    # A* 플래너 호환 메서드
    # ============================================================
    
    def set_obstacles(self, obstacles):
        """A* 호환: 장애물 설정 (D* Lite에서는 costmap 사용)"""
        # D* Lite는 costmap 기반이므로 이 메서드는 호환성을 위해 존재
        pass
    
    def grid_index_to_world(self, gx: int, gz: int) -> Tuple[float, float]:
        """A* 호환: 그리드 → 월드 변환"""
        return self._grid_to_world(gx, gz)
    
    def world_to_grid_index(self, world_x: float, world_z: float) -> Tuple[int, int]:
        """A* 호환: 월드 → 그리드 변환"""
        return self._world_to_grid(world_x, world_z)
    
    # 시각화용 속성
    @property
    def _obstacles(self):
        """A* 호환: 장애물 리스트 (시각화용)"""
        return []
    
    def plot(self, path, current_pos, current_yaw, trajectory=None, 
             title="D* Lite Path", filename="dstar_path.png", show_grid=True):
        """A* 호환: 경로 시각화"""
        try:
            import matplotlib.pyplot as plt
            
            fig, ax = plt.subplots(figsize=(10, 10))
            
            # Costmap 시각화
            if self.costmap is not None and self.costmap_origin is not None:
                extent = [
                    self.costmap_origin[0],
                    self.costmap_origin[0] + self.costmap.shape[1] * self.cell_size,
                    self.costmap_origin[1],
                    self.costmap_origin[1] + self.costmap.shape[0] * self.cell_size
                ]
                ax.imshow(self.costmap, origin='lower', extent=extent, 
                         cmap='RdYlGn_r', alpha=0.5, vmin=0, vmax=1)
            
            # 경로 그리기
            if path:
                xs = [p[0] for p in path]
                zs = [p[1] for p in path]
                ax.plot(xs, zs, 'b-', linewidth=2, label='D* Lite Path')
                ax.plot(xs[-1], zs[-1], 'r*', markersize=15, label='Goal')
            
            # DWA 궤적 그리기
            if trajectory is not None and len(trajectory) > 0:
                tx = trajectory[:, 0]
                ty = trajectory[:, 1]
                ax.plot(tx, ty, 'r--', linewidth=2, label='DWA Trajectory')
            
            # 현재 위치 및 방향
            if current_pos:
                cx, cz = current_pos
                ax.plot(cx, cz, 'go', markersize=10, label='Robot')
                
                if current_yaw is not None:
                    import math
                    arrow_len = 5.0
                    dx = math.sin(math.radians(current_yaw)) * arrow_len
                    dy = math.cos(math.radians(current_yaw)) * arrow_len
                    ax.arrow(cx, cz, dx, dy, head_width=2, head_length=2, 
                            fc='lime', ec='lime')
            
            ax.set_xlim(self.grid_min_x, self.grid_max_x)
            ax.set_ylim(self.grid_min_z, self.grid_max_z)
            ax.set_aspect('equal')
            ax.set_title(title)
            ax.legend()
            
            if show_grid:
                ax.grid(True, linestyle='--', alpha=0.3)
            
            if filename:
                plt.savefig(filename)
                plt.close(fig)
            else:
                plt.show()
                
        except ImportError:
            print("⚠️ matplotlib not available for visualization")
