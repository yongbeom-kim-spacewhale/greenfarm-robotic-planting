"""통합 대시보드의 상태 모델을 제공한다."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

from robot_tests.integrated import config


@dataclass
class TrayState:
    """대시보드에 표시되고 저장되는 한 트레이의 작업 흐름 상태이다.

    이 객체는 로봇을 직접 움직이지 않는다. 로봇의 측정 결과와 다음에
    제시할 작업을 대시보드가 기록하는 데 사용한다.
    """
    soil_status: str = config.INITIAL_SOIL_STATUS
    work_phase: str = config.WORK_PHASE_IDLE
    last_error: str = ""
    last_measurement: dict[str, Any] = field(default_factory=dict)


class IntegratedDashboardLogic:
    """대시보드 위젯과 ROS 프로세스 콜백이 공유하는 상태 머신이다.

    UI 처리 함수는 로봇 노드 실행 전후에 이 모델을 갱신한다. 위젯 활성화,
    트레이 색상, 다음 작업 결정은 모두 이 상태에서 도출되므로 화면 표시
    계층과 로봇 프로세스 제어가 분리된다.
    """

    STATE_PATH = Path.home() / ".config" / "robot_tests" / "integrated_dashboard_state.json"

    # 대시보드의 실행 상태를 초기화하고 저장된 툴·트레이 상태를 복원한다.
    def __init__(self) -> None:
        self.system_started = False
        self.current_tool_state = config.TOOL_STATE_NONE
        self.is_busy = False
        self.is_emergency_stopped = False
        self.is_safety_stopped = False
        self.requires_home_after_estop = False
        self.current_running_task = ""
        self.current_running_tray = ""
        self.current_process = None
        self.ros_connected = False
        self.last_error_message = ""

        self.tray_state = {
            tray_label: TrayState()
            for tray_label in config.TRAY_LABELS
        }
        self._load_persistent_state()

    # 이전 실행에서 저장한 툴 및 트레이 작업 상태를 파일에서 불러온다.
    def _load_persistent_state(self) -> None:
        try:
            payload = json.loads(self.STATE_PATH.read_text(encoding="utf-8"))
        except (FileNotFoundError, OSError, ValueError, TypeError):
            return

        saved_tool = payload.get("current_tool_state")
        # 진행 중 상태가 저장된 채 앱이 종료됐다면 실제 그립 여부를 보장할 수 없다.
        if saved_tool in config.TOOL_TRANSITION_STATES:
            saved_tool = config.TOOL_STATE_UNKNOWN
        if saved_tool in {
            config.TOOL_STATE_NONE,
            config.TOOL_STATE_PRESS,
            config.TOOL_STATE_SHOVEL,
            config.TOOL_STATE_UNKNOWN,
        }:
            self.current_tool_state = saved_tool

        saved_trays = payload.get("tray_state", {})
        if not isinstance(saved_trays, dict):
            return
        for tray_label in config.TRAY_LABELS:
            saved = saved_trays.get(tray_label)
            if not isinstance(saved, dict):
                continue
            self.tray_state[tray_label] = TrayState(
                soil_status=str(saved.get("soil_status", config.INITIAL_SOIL_STATUS)),
                work_phase=str(saved.get("work_phase", config.WORK_PHASE_IDLE)),
                last_error=str(saved.get("last_error", "")),
                last_measurement=dict(saved.get("last_measurement", {})),
            )

    # 작업 중인 툴과 트레이 상태를 설정 파일에 안전하게 저장한다.
    def save_persistent_state(self) -> None:
        payload = {
            "current_tool_state": self.current_tool_state,
            "tray_state": {
                tray_label: asdict(tray)
                for tray_label, tray in self.tray_state.items()
            },
        }
        self.STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.STATE_PATH.with_suffix(".tmp")
        temporary_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary_path.replace(self.STATE_PATH)

    # 로봇이 현재 장착한 것으로 판단되는 툴 상태를 갱신한다.
    def set_tool_state(self, tool_state: str) -> None:
        self.current_tool_state = tool_state
        self.save_persistent_state()

    # 지정한 트레이의 작업 상태를 최초 측정 전 상태로 되돌린다.
    def reset_tray(self, tray_label: str) -> None:
        self.tray_state[tray_label] = TrayState()
        self.save_persistent_state()

    # 지정한 트레이에 새 토양 측정 상태를 기록한다.
    def set_soil_status(self, tray_label: str, soil_status: str) -> None:
        tray = self.tray_state[tray_label]
        tray.soil_status = soil_status
        # 새 측정 결과가 들어오면 이전 보정/평탄화 phase를 종료한다.
        tray.work_phase = config.WORK_PHASE_IDLE
        self.save_persistent_state()

    # 보정 작업과 진단에 사용할 최근 토양 측정 원본 데이터를 저장한다.
    def set_last_measurement(self, tray_label: str, result: dict[str, Any]) -> None:
        if tray_label in self.tray_state:
            self.tray_state[tray_label].last_measurement = dict(result)
            self.save_persistent_state()

    # 트레이 평탄화 완료를 기록하고 다음 재측정이 가능한 상태로 전환한다.
    def mark_flatten_completed(self, tray_label: str) -> None:
        tray = self.tray_state[tray_label]
        # 평탄화 과정에서 새로 측정된 토양 상태를 유지한다.
        tray.work_phase = config.WORK_PHASE_IDLE
        self.save_persistent_state()

    # 흙 추가·제거 또는 돌 제거 완료 후 다음 단계를 평탄화로 전환한다.
    def mark_corrective_action_completed(self, tray_label: str) -> None:
        tray = self.tray_state[tray_label]
        tray.work_phase = config.WORK_PHASE_AFTER_CORRECTION
        self.save_persistent_state()

    # 식물 심기 완료를 기록하고 해당 트레이 작업 흐름을 완료 처리한다.
    def mark_plant_completed(self, tray_label: str) -> None:
        tray = self.tray_state[tray_label]
        tray.soil_status = config.SOIL_STATUS_OK
        tray.work_phase = config.WORK_PHASE_DONE
        self.save_persistent_state()

    # 현재 트레이 상태를 바탕으로 다음에 실행할 로봇 작업을 결정한다.
    def get_next_action(self, tray_label: str) -> str:
        tray = self.tray_state[tray_label]
        if tray.work_phase == config.WORK_PHASE_DONE:
            return config.NEXT_ACTION_DONE
        if tray.work_phase == config.WORK_PHASE_AFTER_CORRECTION:
            return config.NEXT_ACTION_FLATTEN
        if tray.soil_status == config.SOIL_STATUS_LOW:
            return config.NEXT_ACTION_ADD_SOIL
        if tray.soil_status == config.SOIL_STATUS_HIGH:
            return config.NEXT_ACTION_REMOVE_SOIL
        if tray.soil_status == config.SOIL_STATUS_OBSTACLE:
            return config.NEXT_ACTION_REMOVE_ROCK
        if tray.soil_status == config.SOIL_STATUS_OK:
            return config.NEXT_ACTION_PLANT
        return config.NEXT_ACTION_MEASURE

    # 전체 대시보드와 해당 트레이에 작업 오류 메시지를 기록한다.
    def set_error(self, tray_label: str, message: str) -> None:
        self.last_error_message = message
        if tray_label in self.tray_state:
            self.tray_state[tray_label].last_error = message
        self.save_persistent_state()

    # 물리 작업 후 더 이상 유효하지 않은 최근 측정 데이터를 삭제한다.
    def clear_last_measurement(self, tray_label: str) -> None:
        if tray_label in self.tray_state:
            self.tray_state[tray_label].last_measurement = {}
            self.save_persistent_state()
