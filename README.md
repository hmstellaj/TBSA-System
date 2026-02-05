# TBSA-System
TBSA (Tank Battlefield Situational Awareness System) Project

## 프로젝트 소개 자료
[WIKI로 이동](https://github.com/hmstellaj/TBSA-System/wiki)

## 프로젝트 구조

```
project/
├── app.py                      # Flask 서버 메인
├── config.py                   # 전역 설정 관리
├── requirements.txt            # 의존성 목록
│
├── controllers/
│   ├── hybrid_controller.py    # SEQ별 분리 제어 (A*+PID, DWA, PPO)
│   └── pid_controller.py       # PID 제어기
│
├── planners/
│   ├── astar_planner.py        # A* 전역 경로 계획
│   ├── dwa_planner.py          # DWA 로컬 플래너
│   └── gyuppo_planner.py       # 통합 PPO 플래너
│
├── utils/
│   ├── state_manager.py        # 전역 상태 관리
│   ├── lidar_logger.py         # LiDAR 파일 모니터링 + Costmap
│   ├── combat_system.py        # 전투 로직 (조준, 발사)
│   └── visualization.py        # 시각화 매니저
│
├── models/
│   ├── best.pt                 # YOLO 모델 (Legacy)
│   ├── cannon.pt               # Cannon 전용 YOLO
│   ├── integrated.pt           # 통합 객체 인식 YOLO
│   ├── ppo.zip                 # PPO 강화학습 모델
│   └── lidar_frame.py          # LiDAR 데이터 처리
│
├── gyuppo_models/              # 추가 PPO 모델들
│   └── cnn/
│       └── withobs_model/      # 장애물 인식 PPO
│
├── static/                     # 정적 파일 (CSS, JS)
└── templates/
    └── monitor.html            # 웹 모니터링 UI
```

## 만든이들

| 이름 | 담당 | Contact |
|------|------|------|
| 이진진 |  | jj95lee@yonsei.ac.kr |
| 김지온 |  |  |
| 김현건 |  |  |
| 김형규 |  | @sincerite0922 |
| 박승훈 |  | tmdgns0213@gmail.com |
| 이주혁 |  |  |
| 이진행 |  | @playlee1026-cyber |
| 주현민 |  | @stellahmj |

## 출처

| 항목 | 출처 |
|------|------|
| 전차 시뮬레이터 | @BANGBAEDONG VALLEY |
| 첨부 이미지 | @GOOGLE GEMINI |
