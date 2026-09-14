# GreenFarm — 접촉 기반 협동로봇 토양 관리·식재 자동화

카메라 없이 협동로봇의 힘·순응제어로 토양 높이와 장애물을 판별하고, 상태별 보정 후 식재까지 수행하는 ROS 2 시스템입니다.

[시연 영상](https://drive.google.com/file/d/1p9_dyek6e7o9HDlVhM4NHQk2XoZrypt7/view)

## 프로젝트 정보

- 기간: 2026.07.01–2026.07.14
- 인원: 3명
- 역할: 팀장, 로봇 제어·통합, 접촉 측정과 상태 분기 설계
- 기술: ROS 2, Python, Doosan M0609, DSR API, PyQt5, 힘·순응제어

## 핵심 성과

- 트레이 4지점 접촉 z값으로 토양 상태를 판정하고 정상·흙부족·흙많음·장애물 상태별 작업을 연결
- 공구 교체, 흙 보정, 평탄화, 재측정, 식재를 Dashboard에서 통합 실행
- 실기 환경에서 좌표·접촉 임계값을 조정하고 긴급정지·HOME 복귀 흐름을 구성

## 주요 기능

- PyQt 기반 통합 Dashboard와 ROS 연결 상태 확인
- Tray A/B/C/D별 토양 상태 측정
- 상태 판정: 미측정, 정상, 흙부족, 흙많음, 장애물감지, 측정실패
- 상태별 흙 추가·제거, 돌 제거, 평탄화와 재측정
- 식물 pick-and-place, 공구 교체, 수동 제어
- 긴급정지와 HOME 복귀

## 실행 환경

- ROS 2
- Doosan Robotics ROS 2 및 DSR API
- Python, PyQt5
- Doosan M0609

환경이 다르면 robot_tests/integrated/config.py의 WORKSPACE_SETUP_PATH를 실제 설치 경로에 맞게 수정해야 합니다.

## 빌드와 실행


auditable commands:

    cd /home/rokey/ws_cobot_pjt/ws_cobot1
    colcon build --packages-select robot_tests
    source install/setup.bash
    ros2 launch robot_tests integrated_dashboard.launch.py

## 핵심 작업 흐름

    Dashboard 실행 → ROS 연결 → Tray 선택 → 4지점 접촉 측정 → 상태 판정
    정상 → 식물 심기 → 완료
    흙부족/흙많음/장애물 → 상태별 보정 → 평탄화 → 재측정
    측정실패 → UI 오류 표시 → 재측정 또는 HOME 복귀

## 저장소 구조

    robot_tests/
    ├─ launch/integrated_dashboard.launch.py
    ├─ robot_tests/integrated/       # Dashboard, 상태와 실행 프로세스 관리
    ├─ robot_tests/standalone_tests/ # 기능 단위 검증
    ├─ robot_tests/motion_primitives.py
    ├─ robot_tests/robot_motion_data.py
    ├─ package.xml
    └─ setup.py

## 주요 노드

| 실행 이름 | 역할 |
|---|---|
| integrated_dashboard | PyQt Dashboard |
| integrated_tray_soil_state_check_node | 접촉 측정과 토양 상태 판정 |
| integrated_tray_next_step_node | 상태 기반 다음 작업 실행 |
| integrated_tray_soil_flatten_node | 선택 Tray 평탄화 |
| integrated_plant_pick_and_place_node | 식물 식재 |
| integrated_manual_control_node | Base/Joint 수동 제어 |
| integrated_emergency_stop_node | 긴급정지 |

## 검증과 주의사항

기능 단위 테스트 후 흙 보정→평탄화→재측정→식재까지 통합 흐름을 실기에서 검증했습니다. 실제 로봇에서 실행하기 전 좌표, Tool/TCP, 접촉 임계값은 현장 환경에 맞게 재확인해야 합니다. 물리 E-Stop 상태를 Dashboard에 직접 반영하는 기능은 추가 개선 대상입니다.
