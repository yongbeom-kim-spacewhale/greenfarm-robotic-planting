"""통합 스마트팜 대시보드의 사용자 인터페이스를 제공한다."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime
import json
from pathlib import Path
import re

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QProcess, QTimer

from robot_tests.integrated import config
from robot_tests.integrated.dashboard_logic import IntegratedDashboardLogic
from robot_tests.integrated.ros_connection_monitor import RobotConnectionMonitor
from robot_tests.integrated.ros_runner import RosNodeRunner
from robot_tests.safety_state import activate_safety_stop, clear_safety_stop


class IntegratedDashboard(QtWidgets.QMainWindow):
    """통합 로봇 작업 흐름의 작업자 UI와 최상위 제어를 담당한다.

    버튼 처리 함수는 실행할 작업을 결정하고 ``RosNodeRunner``는 자식
    프로세스를 관리하며, 완료 및 출력 콜백은 ``IntegratedDashboardLogic``을
    갱신한다. 실제 로봇 동작은 ROS 노드가 수행하고 이 클래스는 실행 순서만
    조정한다.
    """

    TRAY_FRAME_STYLES = {
        config.SOIL_STATUS_UNKNOWN: ("#f3f4f6", "#9ca3af"),
        config.SOIL_STATUS_OK: ("#f0fdf4", "#6bbf7a"),
        config.SOIL_STATUS_LOW: ("#fffbea", "#e0b84f"),
        config.SOIL_STATUS_HIGH: ("#fffbea", "#e0b84f"),
        config.SOIL_STATUS_OBSTACLE: ("#fff1f2", "#ef6b73"),
        config.SOIL_STATUS_FAILED: ("#fff1f2", "#ef6b73"),
    }

    TRAY_STATE_TEXT_COLORS = {
        config.SOIL_STATUS_UNKNOWN: "#6b7280",
        config.SOIL_STATUS_OK: "#0b6b20",
        config.SOIL_STATUS_LOW: "#b7791f",
        config.SOIL_STATUS_HIGH: "#b7791f",
        config.SOIL_STATUS_OBSTACLE: "#d60000",
        config.SOIL_STATUS_FAILED: "#d60000",
    }

    # UI와 상태 모델을 초기화하고 ROS 연결 및 수동 제어 감시를 시작한다.
    def __init__(self, ui_path: str | None = None) -> None:
        super().__init__()
        self.logic = IntegratedDashboardLogic()
        self.runner = RosNodeRunner(self)
        self.pending_task: tuple[str, str] | None = None
        self.estop_state_snapshot: dict | None = None
        self.safety_stop_state_snapshot: dict | None = None
        self.physical_recovery_ready = False
        self.last_robot_state: int | None = None
        self.recovery_command_pending = False
        self.recovery_attempt_id = 0
        self.singularity_notice_shown = False
        self.developer_log_path = (
            Path.home()
            / ".local"
            / "state"
            / "robot_tests"
            / "integrated_dashboard_developer.log"
        )
        self._prepare_developer_log()

        # 감시 스레드는 실제 연결 여부만 판정하고, UI 변경은 아래 Qt signal을 통해
        # 메인 스레드의 handle_robot_connection_changed()에서 처리한다.
        self.robot_link_available = False
        self.connection_monitor = RobotConnectionMonitor(
            self,
            joint_state_topic=config.ROBOT_JOINT_STATE_TOPIC,
            disconnection_topic=config.ROBOT_DISCONNECTION_TOPIC,
            error_topic=config.ROBOT_ERROR_TOPIC,
            robot_state_service=config.ROBOT_STATE_SERVICE,
            robot_control_service=config.ROBOT_CONTROL_SERVICE,
            heartbeat_timeout_seconds=config.ROBOT_HEARTBEAT_TIMEOUT_SECONDS,
            reconnect_message_count=config.ROBOT_RECONNECT_MESSAGE_COUNT,
            state_poll_interval_seconds=config.ROBOT_STATE_POLL_INTERVAL_SECONDS,
            state_request_timeout_seconds=config.ROBOT_STATE_REQUEST_TIMEOUT_SECONDS,
            safe_off_reset_control=config.ROBOT_CONTROL_RESET_SAFE_OFF,
        )
        self.connection_monitor.connection_changed.connect(
            self.handle_robot_connection_changed
        )
        self.connection_monitor.robot_error_received.connect(self.handle_robot_error)
        self.connection_monitor.robot_state_changed.connect(self.handle_robot_state_changed)
        self.connection_monitor.safe_off_reset_finished.connect(
            self.handle_safe_off_reset_finished
        )

        resolved_ui_path = self.resolve_ui_path(ui_path)
        if resolved_ui_path is None:
            raise FileNotFoundError("No SmartFarm dashboard .ui file found")

        uic.loadUi(str(resolved_ui_path), self)
        self.initialize_state()
        self.connect_signals()
        self.write_log("대시보드 준비 완료")
        self.connection_monitor.start()
        self.runner.start_manual_server()

    # 지정 경로와 기본 후보에서 사용할 Qt Designer UI 파일을 찾는다.
    def resolve_ui_path(self, ui_path: str | None) -> Path | None:
        candidates = []
        if ui_path:
            candidates.append(Path(ui_path))

        current_dir = Path(__file__).resolve().parent
        candidates.append(current_dir / "smartfarm_dashboard.ui")

        for candidate in config.UI_PATH_CANDIDATES:
            candidates.append(Path(candidate))

        for candidate in candidates:
            if candidate.is_file():
                return candidate
        return None

    # 로봇 작업 중에는 창 종료를 막고 대기 상태에서만 종료를 허용한다.
    def closeEvent(self, event) -> None:
        if self.logic.is_busy:
            event.ignore()
            self.write_log("로봇 동작 중 UI 종료 요청 차단")
            QtWidgets.QMessageBox.warning(
                self,
                "종료할 수 없음",
                "로봇 동작이 진행 중입니다. 작업이 완료된 후 UI를 종료하세요.",
            )
            return
        # ROS executor가 남은 상태로 Qt 객체가 파괴되지 않도록 먼저 감시를 종료한다.
        self.connection_monitor.stop()
        self.connection_monitor.wait(2000)
        event.accept()

    # 복원된 상태를 화면에 반영하고 최초 버튼 활성화 상태를 설정한다.
    def initialize_state(self) -> None:
        # 새 대시보드 세션은 정지 상태가 아니므로 이전 비정상 종료의 플래그를 정리한다.
        clear_safety_stop()
        self.set_label("lbl_robot_status", "대기 중")
        self.set_label("lbl_ros_status", "Disconnected")

        # 이 버튼은 서보를 끄지 않고 MoveStop(DR_QSTOP)을 호출하는 소프트웨어
        # 안전정지다. 물리 비상정지는 robot_state 감시로 별도 처리한다.
        estop_button = getattr(self, "btn_estop", None)
        if estop_button is not None:
            estop_button.setText("안전정지")
            estop_button.show()

        for tray_label in config.TRAY_LABELS:
            self.update_tray_widgets(tray_label)

        if hasattr(self, "txt_log"):
            self.txt_log.clear()
            self.txt_log.setReadOnly(True)

        self.refresh_button_state()

    # 대시보드의 모든 조작 버튼을 해당 작업 처리 함수에 연결한다.
    def connect_signals(self) -> None:
        self.connect_button("btn_start", self.start_system)
        self.connect_button("btn_shutdown", self.shutdown_system)
        self.connect_button("btn_home", self.run_home)
        self.connect_button("btn_estop", self.emergency_stop)
        self.connect_button("btn_recovery", self.recover_from_estop)
        self.connect_button("btn_press_pickup", self.run_press_pickup)
        self.connect_button("btn_press_putdown", self.run_press_putdown)
        self.connect_button("btn_shovel_pickup", self.run_shovel_pickup)
        self.connect_button("btn_shovel_putdown", self.run_shovel_putdown)
        self.connect_button("btn_move_joint", self.run_move_joint)
        self.connect_button("btn_read_joint", self.run_read_joint)
        self.connect_button("btn_move_base", self.run_move_base)
        self.connect_button("btn_read_base", self.run_read_base)

        for index, tray_label in enumerate(config.TRAY_LABELS, start=1):
            self.connect_button(
                f"btn_tray{index}_check",
                lambda _checked=False, tray=tray_label: self.run_measure(tray),
            )
            self.connect_button(
                f"btn_tray{index}_next",
                lambda _checked=False, tray=tray_label: self.run_next_action(tray),
            )
            self.connect_button(
                f"btn_tray{index}_reset",
                lambda _checked=False, tray=tray_label: self.reset_tray(tray),
            )

    # UI에 버튼이 존재할 때만 클릭 이벤트 처리 함수를 연결한다.
    def connect_button(self, object_name: str, callback) -> None:
        button = getattr(self, object_name, None)
        if button is not None:
            button.clicked.connect(callback)

    # UI에 라벨이 존재할 때 표시 문구를 갱신한다.
    def set_label(self, object_name: str, text: str) -> None:
        label = getattr(self, object_name, None)
        if label is not None:
            label.setText(str(text))

    # 개발자 로그 파일을 준비하고 지나치게 커진 이전 파일은 한 번 회전시킨다.
    def _prepare_developer_log(self) -> None:
        try:
            self.developer_log_path.parent.mkdir(parents=True, exist_ok=True)
            if (
                self.developer_log_path.exists()
                and self.developer_log_path.stat().st_size > 5 * 1024 * 1024
            ):
                rotated_path = self.developer_log_path.with_suffix(".log.1")
                self.developer_log_path.replace(rotated_path)
            with self.developer_log_path.open("a", encoding="utf-8") as log_file:
                log_file.write(
                    f"\n===== dashboard session {datetime.now().isoformat(timespec='seconds')} =====\n"
                )
        except OSError:
            # 로그 파일 문제로 대시보드 자체가 실행되지 않는 상황은 피한다.
            pass

    # ROS 원문, 내부 상태, 서비스 결과를 UI와 분리된 파일에 저장한다.
    def write_developer_log(self, text: str, *, level: str = "DEBUG") -> None:
        timestamp = datetime.now().isoformat(timespec="milliseconds")
        try:
            with self.developer_log_path.open("a", encoding="utf-8") as log_file:
                log_file.write(f"[{timestamp}] [{level}] {text}\n")
        except OSError:
            pass

    # 작업자가 알아야 하는 간단한 메시지만 UI 로그 창에 표시한다.
    def write_log(self, text: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        message = f"[{timestamp}] {text}"
        if hasattr(self, "txt_log"):
            self.txt_log.append(message)
        self.write_developer_log(f"USER: {text}", level="INFO")

    # 내부 작업 식별자를 사용자에게 익숙한 작업명으로 변환한다.
    def user_task_name(self, task_name: str) -> str:
        return {
            "PRESS_PLATE_PICKUP": "누름판 집기",
            "PRESS_PLATE_PUTDOWN": "누름판 놓기",
            "SHOVEL_PICKUP": "삽 집기",
            "SHOVEL_PUTDOWN": "삽 놓기",
            "HOME": "초기 위치 이동",
            "READ_JOINT": "관절값 읽기",
            "READ_BASE": "로봇 끝단 위치 읽기",
            "MOVE_JOINT": "관절 이동",
            "MOVE_BASE": "로봇 끝단 이동",
            "RECOVER_TOOL_TCP": "로봇 도구 설정 복구",
        }.get(task_name, task_name or "로봇 작업")

    # 감시 스레드가 판정한 실제 로봇 연결 상태를 UI와 버튼 상태에 반영한다.
    def handle_robot_connection_changed(self, connected: bool, reason: str) -> None:
        self.robot_link_available = connected
        self.write_developer_log(
            f"connection_changed connected={connected}, reason={reason}",
            level="INFO",
        )

        # 시스템 시작 전에는 실제 연결을 기억만 하고 화면은 Disconnected로 유지한다.
        if not self.logic.system_started:
            return

        self.write_log(f"로봇 연결 {'확인' if connected else '끊김'}")
        was_connected = self.logic.ros_connected
        self.logic.ros_connected = connected
        self.set_label("lbl_ros_status", "Connected" if connected else "Disconnected")
        self.refresh_button_state()

        # Connected에서 실제로 끊긴 순간에만 경고하여 같은 팝업이 반복되지 않게 한다.
        if was_connected and not connected:
            QtWidgets.QMessageBox.warning(
                self,
                "로봇 연결 끊김",
                "실제 로봇 연결이 끊겼습니다. 연결 상태를 확인하세요.",
            )

    # /dsr01/error의 원본 필드를 로그에 남겨 현장에서 안전 오류 코드를 확인한다.
    def handle_robot_error(self, level: int, group: int, code: int, details: str) -> None:
        self.write_developer_log(
            f"ROBOT_ERROR level={level} group={group} code={code} msg={details or '-'}",
            level="ERROR",
        )

        # 측정/평탄화의 순응제어 해제 직후 발생하는 1903은 실제 접촉 Z값과 함께
        # 개발자 로그에만 보관한다. 그 밖의 오류는 기존처럼 작업자에게 알린다.
        expected_compliance_release = group == 2 and code == 1903
        compliance_task_running = self.logic.current_running_task in {
            config.NEXT_ACTION_MEASURE,
            config.NEXT_ACTION_FLATTEN,
        }
        if expected_compliance_release and compliance_task_running:
            return

        # 3205/3206은 특이점 영역의 진입/이탈 상태 알림이다. 원본 이벤트는
        # 모두 개발자 로그에 남기되 작업자 UI에는 한 작업당 한 번만 표시한다.
        if group == 2 and code in {3205, 3206}:
            if not self.singularity_notice_shown:
                self.write_log("특이점 구간 이동 중 - 로봇 자세를 전환하고 있습니다")
                self.singularity_notice_shown = True
            return

        self.write_log("로봇 오류 감지 - 로봇 상태와 안전을 확인하세요")

    # 상태 서비스의 EMERGENCY_STOP과 STANDBY 전환으로 물리 복구 단계를 판정한다.
    def handle_robot_state_changed(self, robot_state: int) -> None:
        self.last_robot_state = robot_state
        self.write_developer_log(f"robot_state changed: {robot_state}", level="INFO")
        if robot_state == config.ROBOT_STATE_EMERGENCY_STOP:
            self.enter_physical_emergency_stop()
            return

        if not self.logic.is_emergency_stopped:
            return
        if robot_state not in config.ROBOT_RECOVERY_READY_STATES:
            self.physical_recovery_ready = False
            self.refresh_button_state()
            return

        if robot_state == config.ROBOT_STATE_STANDBY and self.recovery_command_pending:
            self.recovery_command_pending = False
            self.write_log("로봇 안전복구 완료")
            self.complete_dashboard_recovery()
            return

        self.physical_recovery_ready = not self.recovery_command_pending
        if robot_state == config.ROBOT_STATE_SAFE_OFF:
            status_text = "서보 연결 해제 - 안전복구 버튼을 눌러주세요"
        else:
            status_text = "물리 안전복구 확인 - 안전복구 버튼을 누르세요"
        self.set_label("lbl_robot_status", status_text)
        self.write_log("안전복구 버튼을 눌러주세요")
        self.refresh_button_state()

    # 물리 비상정지 직전의 논리 상태를 저장하고 실행 입력을 모두 잠근다.
    def enter_physical_emergency_stop(self) -> None:
        if self.logic.is_emergency_stopped:
            self.physical_recovery_ready = False
            self.refresh_button_state()
            return

        self.resolve_interrupted_tool_state()
        self.estop_state_snapshot = {
            "system_started": self.logic.system_started,
            "tool_state": self.logic.current_tool_state,
            "tray_state": deepcopy(self.logic.tray_state),
        }
        self.logic.is_emergency_stopped = True
        self.logic.requires_home_after_estop = True
        self.physical_recovery_ready = False
        self.recovery_command_pending = False
        self.recovery_attempt_id += 1
        self.pending_task = None
        activate_safety_stop()
        self.runner.stop_current_process()
        self.logic.is_busy = False
        self.logic.current_running_task = ""
        self.logic.current_running_tray = ""
        self.logic.current_process = None
        self.set_label("lbl_robot_status", "물리 비상정지")
        self.write_log("물리 비상정지 감지 - 대시보드 상태 저장 및 전체 잠금")
        self.refresh_button_state()
        QtWidgets.QMessageBox.critical(
            self,
            "물리 비상정지",
            "로봇의 물리 비상정지가 감지되었습니다.\n상황을 확인하고 물리 안전복구를 진행하세요.",
        )

    # ROS 노드 사이의 안정화 시간을 기다린 뒤 후속 작업을 실행한다.
    def run_after_inter_node_delay(
        self,
        callback,
        *,
        reason: str = "",
        delay_ms: int | None = None,
    ) -> None:
        delay_ms = int(config.INTER_NODE_DELAY_MS if delay_ms is None else delay_ms)
        if reason:
            self.write_developer_log(f"inter-node delay: {reason}, {delay_ms}ms")

        # 노드 사이 대기 중 안전정지가 발생해도 예약된 다음 동작이 시작되지 않게 한다.
        def _run_if_motion_is_allowed() -> None:
            if (
                self.logic.is_safety_stopped
                or self.logic.is_emergency_stopped
                or self.logic.requires_home_after_estop
            ):
                self.write_log("정지 상태로 인해 예약된 후속 작업을 취소함")
                return
            callback()

        QTimer.singleShot(delay_ms, _run_if_motion_is_allowed)

    # 소프트웨어 안전정지 사실을 알리고 작업자의 확인을 요구한다.
    def show_emergency_popup(self) -> None:
        message_box = QtWidgets.QMessageBox(self)
        message_box.setIcon(QtWidgets.QMessageBox.Warning)
        message_box.setWindowTitle("안전정지")
        message_box.setText("로봇 이동을 안전정지했습니다. 상황을 확인하세요.")
        message_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
        message_box.setDefaultButton(QtWidgets.QMessageBox.Ok)
        message_box.exec_()

    # 지정 트레이의 상태, 다음 작업, 버튼 문구와 색상을 갱신한다.
    def update_tray_widgets(self, tray_label: str) -> None:
        tray = self.logic.tray_state[tray_label]
        idx = config.TRAY_LABELS.index(tray_label) + 1
        next_action = self.logic.get_next_action(tray_label)
        self.set_label(f"lbl_tray{idx}_state", tray.soil_status)
        self.set_label(f"lbl_tray{idx}_next_action", next_action)
        next_button = getattr(self, f"btn_tray{idx}_next", None)
        if next_button is not None:
            is_initial_state = (
                tray.soil_status == config.SOIL_STATUS_UNKNOWN
                and tray.work_phase == config.WORK_PHASE_IDLE
                and not tray.last_measurement
            )
            next_button.setText(
                config.NEXT_BUTTON_DEFAULT_TEXT if is_initial_state else next_action
            )
        self.update_tray_frame_color(idx, tray.soil_status)
        self.update_tray_state_text_color(idx, tray.soil_status)

    # 토양 상태의 심각도에 맞춰 트레이 프레임 색상을 변경한다.
    def update_tray_frame_color(self, idx: int, soil_status: str) -> None:
        frame = getattr(self, f"frame_tray{idx}", None)
        if frame is None:
            return

        background, border = self.TRAY_FRAME_STYLES.get(
            soil_status,
            self.TRAY_FRAME_STYLES[config.SOIL_STATUS_UNKNOWN],
        )
        frame.setStyleSheet(
            f"QFrame{{background:{background}; border:1px solid {border}; border-radius:12px;}}"
        )

    # 토양 상태에 맞춰 트레이 상태 글자의 색상을 변경한다.
    def update_tray_state_text_color(self, idx: int, soil_status: str) -> None:
        label = getattr(self, f"lbl_tray{idx}_state", None)
        if label is None:
            return

        color = self.TRAY_STATE_TEXT_COLORS.get(soil_status, "#1f2937")
        label.setStyleSheet(f"border:none; font-size:15px; font-weight:800; color:{color};")

    # 시스템·작업·긴급정지·툴·ROS 상태에 따라 버튼 사용 가능 여부를 정한다.
    def refresh_button_state(self) -> None:
        # 물리 비상정지 중에는 전부 잠그고, STANDBY 복귀가 확인된 뒤에만
        # 대시보드 안전복구 버튼 하나를 허용한다.
        if self.logic.is_emergency_stopped:
            enabled = {"btn_recovery"} if self.physical_recovery_ready else set()
            for button in self.findChildren(QtWidgets.QPushButton):
                button.setEnabled(button.objectName() in enabled)
            return

        # 소프트웨어 안전정지 직후에는 HOME 이동도 공유 플래그로 차단되어 있으므로
        # 안전복구 버튼만 허용한다. 복구가 플래그를 해제한 뒤 HOME을 활성화한다.
        if self.logic.is_safety_stopped:
            enabled = {"btn_recovery"}
            for button in self.findChildren(QtWidgets.QPushButton):
                button.setEnabled(button.objectName() in enabled)
            return

        if not self.logic.system_started:
            enabled = {"btn_start"}
            for button in self.findChildren(QtWidgets.QPushButton):
                button.setEnabled(button.objectName() in enabled)
            return

        busy = self.logic.is_busy
        requires_home = self.logic.requires_home_after_estop
        tool_state = self.logic.current_tool_state
        ros_connected = self.logic.ros_connected

        # 자동/수동 로봇 작업이 실행 중일 때는 중복 명령이나 상태 변경이 들어오지
        # 않도록 화면의 모든 버튼을 잠근다. 작업 종료/실패 처리에서 is_busy가
        # 해제되면 현재 시스템 상태에 맞는 버튼들이 다시 활성화된다.
        if busy:
            # 실행 중에도 별도 보조 프로세스의 MoveStop을 호출할 수 있어야 한다.
            enabled = {"btn_estop"}
        elif requires_home:
            # 안전복구 직후에는 다른 명령을 허용하지 않고 HOME 복귀만 요구한다.
            # HOME 이동이 시작되면 busy 분기에서 안전정지 버튼이 다시 활성화된다.
            enabled = {"btn_home"}
        else:
            enabled = {
                "btn_shutdown",
                "btn_estop",
            }
            if ros_connected and not busy:
                enabled.update(
                    {
                        "btn_home",
                        "btn_move_joint",
                        "btn_read_joint",
                        "btn_move_base",
                        "btn_read_base",
                        "btn_press_putdown",
                        "btn_shovel_putdown",
                    }
                )
                if tool_state in (config.TOOL_STATE_NONE, config.TOOL_STATE_UNKNOWN):
                    enabled.update(
                        {
                            "btn_press_pickup",
                            "btn_shovel_pickup",
                        }
                    )
                for idx in range(1, 5):
                    tray_label = config.TRAY_LABELS[idx - 1]
                    enabled.add(f"btn_tray{idx}_reset")
                    if self.logic.get_next_action(tray_label) == config.NEXT_ACTION_DONE:
                        continue
                    enabled.add(f"btn_tray{idx}_check")
                    tray = self.logic.tray_state[tray_label]
                    if tray.soil_status not in (
                        config.SOIL_STATUS_UNKNOWN,
                        config.SOIL_STATUS_FAILED,
                    ):
                        enabled.add(f"btn_tray{idx}_next")

        for button in self.findChildren(QtWidgets.QPushButton):
            button.setEnabled(button.objectName() in enabled)

    # 로봇을 한 작업에 할당하고 충돌할 수 있는 사용자 입력을 잠근다.
    def lock_for_task(self, task_name: str, tray_label: str) -> None:
        self.logic.is_busy = True
        self.logic.current_running_task = task_name
        self.logic.current_running_tray = tray_label
        self.singularity_notice_shown = False
        user_task = self.user_task_name(task_name)
        tray_prefix = f"{tray_label} 트레이 " if tray_label in config.TRAY_LABELS else ""
        self.set_label("lbl_robot_status", f"{tray_prefix}{user_task} 실행 중")
        self.refresh_button_state()

    # 작업 잠금을 해제하고 현재 상태에 맞게 화면 제어를 복원한다.
    def unlock_after_task(self) -> None:
        self.logic.is_busy = False
        self.logic.current_running_task = ""
        self.logic.current_running_tray = ""
        if self.logic.requires_home_after_estop:
            self.set_label("lbl_robot_status", "초기 위치 이동 필요")
            self.refresh_button_state()
            return
        self.set_label("lbl_robot_status", "대기 중")
        self.refresh_button_state()

    # 통합 툴 제어 노드로 지정 툴의 장착 또는 반환 작업을 시작한다.
    def start_tool_node(
        self,
        tool: str,
        action: str,
        *,
        skip_initial_home: bool = False,
    ) -> None:
        transition_state = {
            ("press", "pickup"): config.TOOL_STATE_PRESS_PICKING_APPROACH,
            ("press", "putdown"): config.TOOL_STATE_PRESS_PUTTING_APPROACH,
            ("shovel", "pickup"): config.TOOL_STATE_SHOVEL_PICKING_APPROACH,
            ("shovel", "putdown"): config.TOOL_STATE_SHOVEL_PUTTING_APPROACH,
        }[(tool, action)]
        self.logic.set_tool_state(transition_state)
        self.write_developer_log(f"tool transition state={transition_state}")
        node_args = ["--tool", tool, "--action", action]
        if skip_initial_home:
            node_args.append("--skip-initial-home")
        self.runner.start_node(config.INTEGRATED_TOOL_NODE, node_args=node_args)

    # 필요 시 누름판을 먼저 장착한 후 지정 트레이의 토양 측정을 시작한다.
    def run_measure(self, tray_label: str) -> None:
        if self.logic.is_busy or not self.logic.ros_connected:
            return
        if self.logic.current_tool_state == config.TOOL_STATE_UNKNOWN:
            self.write_log("현재 툴을 확인할 수 없어 토양 측정을 시작할 수 없습니다")
            return
        if self.logic.current_tool_state == config.TOOL_STATE_SHOVEL:
            self.pending_task = ("measure_press_pickup", tray_label)
            self.lock_for_task("SHOVEL_PUTDOWN", tray_label)
            self.write_log("토양 측정 전 삽을 반환합니다")
            self.start_tool_node("shovel", "putdown")
            return
        if self.logic.current_tool_state != config.TOOL_STATE_PRESS:
            self.pending_task = ("measure", tray_label)
            self.lock_for_task("PRESS_PLATE_PICKUP", tray_label)
            self.write_log("토양 측정용 누름판을 장착합니다")
            self.start_tool_node("press", "pickup")
            return
        self._start_measure_task(tray_label)

    # 누름판 준비가 끝난 상태에서 실제 토양 측정 노드를 실행한다.
    def _start_measure_task(self, tray_label: str) -> None:
        self.lock_for_task(config.NEXT_ACTION_MEASURE, tray_label)
        self.write_log(f"{tray_label} 트레이 토양 측정 시작")
        self.runner.start_node(
            config.MEASURE_NODE,
            node_args=[
                "--tray",
                tray_label,
                "--skip-initial-home",
            ],
        )

    # 툴을 정리하고 흙 추가·제거 또는 돌 제거 보정 작업을 시작한다.
    def run_corrective_action(self, tray_label: str) -> None:
        if self.logic.is_busy or not self.logic.ros_connected:
            return
        if self.logic.current_tool_state == config.TOOL_STATE_UNKNOWN:
            self.write_log("현재 툴을 확인할 수 없어 보정 작업을 시작할 수 없습니다")
            return
        if self.logic.current_tool_state == config.TOOL_STATE_PRESS:
            self.pending_task = ("next", tray_label)
            self.lock_for_task("PRESS_PLATE_PUTDOWN", tray_label)
            self.write_log("보정 작업 전 누름판을 반환합니다")
            self.start_tool_node("press", "putdown", skip_initial_home=True)
            return
        # 대시보드의 이전 자동 작업은 HOME 복귀 후 종료한다. 보정 노드에서 다시
        # TCP를 120 mm 상승시키면 불필요한 중복 동작과 위치 조회 대기가 발생한다.
        self._start_next_step_task(tray_label, skip_initial_home=True)

    # 누름판 장착 상태를 확보한 뒤 지정 트레이의 평탄화 흐름을 시작한다.
    def run_flatten_action(self, tray_label: str) -> None:
        if self.logic.is_busy or not self.logic.ros_connected:
            return
        if self.logic.current_tool_state == config.TOOL_STATE_UNKNOWN:
            self.write_log("현재 툴을 확인할 수 없어 평탄화를 시작할 수 없습니다")
            return
        if self.logic.current_tool_state == config.TOOL_STATE_SHOVEL:
            self.pending_task = ("flatten_press_pickup", tray_label)
            self.lock_for_task("SHOVEL_PUTDOWN", tray_label)
            self.write_log("평탄화 전 삽을 반환합니다")
            self.start_tool_node("shovel", "putdown")
            return
        tray = self.logic.tray_state[tray_label]
        if tray.work_phase == config.WORK_PHASE_AFTER_CORRECTION:
            # 보정 노드는 삽을 반환하고 빈 그리퍼로 끝나는 것이 정상 흐름이다.
            # 이전 상태가 잘못 남았더라도 평탄화 전에 누름판 pickup을 확실히 수행한다.
            self.logic.set_tool_state(config.TOOL_STATE_NONE)
            self.pending_task = ("flatten", tray_label)
            self.lock_for_task("PRESS_PLATE_PICKUP", tray_label)
            self.write_log("평탄화용 누름판을 장착합니다")
            self.start_tool_node("press", "pickup")
            return
        if self.logic.current_tool_state != config.TOOL_STATE_PRESS:
            self.pending_task = ("flatten", tray_label)
            self.lock_for_task("PRESS_PLATE_PICKUP", tray_label)
            self.write_log("평탄화용 누름판을 장착합니다")
            self.start_tool_node("press", "pickup")
            return
        self._start_flatten_task(tray_label)

    # 지정 트레이에 대한 컴플라이언스 기반 누름 평탄화 노드를 실행한다.
    def _start_flatten_task(self, tray_label: str) -> None:
        self.lock_for_task(config.NEXT_ACTION_FLATTEN, tray_label)
        self.write_log(f"{tray_label} 트레이 평탄화 시작")
        self.runner.start_node(
            config.INTEGRATED_FLATTEN_NODE,
            node_args=["--tray", tray_label],
        )

    # 트레이 상태가 요구하는 다음 작업을 알맞은 실행 흐름으로 분기한다.
    def run_next_action(self, tray_label: str) -> None:
        next_action = self.logic.get_next_action(tray_label)
        if next_action == config.NEXT_ACTION_MEASURE:
            self.run_measure(tray_label)
        elif next_action in (
            config.NEXT_ACTION_ADD_SOIL,
            config.NEXT_ACTION_REMOVE_SOIL,
            config.NEXT_ACTION_REMOVE_ROCK,
        ):
            self.run_corrective_action(tray_label)
        elif next_action == config.NEXT_ACTION_FLATTEN:
            self.run_flatten_action(tray_label)
        elif next_action == config.NEXT_ACTION_PLANT:
            if self.logic.is_busy or not self.logic.ros_connected:
                return
            if self.logic.current_tool_state == config.TOOL_STATE_UNKNOWN:
                self.write_log("현재 툴을 확인할 수 없어 식물 심기를 시작할 수 없습니다")
                return
            if self.logic.current_tool_state == config.TOOL_STATE_PRESS:
                self.pending_task = ("next", tray_label)
                self.lock_for_task("PRESS_PLATE_PUTDOWN", tray_label)
                self.write_log("식물 심기 전 누름판을 반환합니다")
                self.start_tool_node("press", "putdown", skip_initial_home=True)
                return
            self._start_next_step_task(tray_label)
        else:
            self.write_log(f"{tray_label} 트레이는 완료 상태")

    # 로봇을 움직이지 않고 지정 트레이의 논리적 진행 상태만 초기화한다.
    def reset_tray(self, tray_label: str) -> None:
        if self.logic.is_busy:
            return
        self.logic.reset_tray(tray_label)
        self.update_tray_widgets(tray_label)
        self.write_log(f"{tray_label} 트레이 리셋 - 초기 상태로 복원")
        self.refresh_button_state()

    # 트레이 상태와 측정값을 인자로 구성하여 통합 후속 작업 노드를 실행한다.
    def _start_next_step_task(
        self,
        tray_label: str,
        *,
        skip_initial_home: bool = False,
    ) -> None:
        tray = self.logic.tray_state[tray_label]
        next_action = self.logic.get_next_action(tray_label)
        node_args = [
            "--tray",
            tray_label,
            "--soil-status",
            tray.soil_status,
            "--next-action",
            next_action,
        ]
        if skip_initial_home:
            node_args.append("--skip-initial-home")
        if next_action == config.NEXT_ACTION_PLANT:
            node_args.extend(["--plant", str(config.TRAY_TO_PLANT_INDEX[tray_label])])
        if tray.last_measurement:
            node_args.extend(
                [
                    "--measurement-json",
                    json.dumps(tray.last_measurement, ensure_ascii=False, separators=(",", ":")),
                ]
            )
        self.lock_for_task(next_action, tray_label)
        self.write_log(f"{tray_label} 트레이 {next_action} 시작")
        self.write_developer_log(
            f"next task args: tray={tray_label}, soil={tray.soil_status}, "
            f"next={next_action}, skip_initial_home={skip_initial_home}"
        )
        self.runner.start_node(config.INTEGRATED_NEXT_STEP_NODE, node_args=node_args)

    # 현재 관절 각도를 요청하여 수동 제어 입력 위젯에 표시한다.
    def run_read_joint(self) -> None:
        if self.logic.is_busy:
            return
        self.lock_for_task("READ_JOINT", "-")
        self.write_log("현재 관절값 읽기")
        self.runner.send_manual_command({"mode": "read-joint"})

    # 화면에 입력된 여섯 관절 각도로 관절 공간 이동을 요청한다.
    def run_move_joint(self) -> None:
        if self.logic.is_busy:
            return
        joints = [getattr(self, f"spin_j{idx}").value() for idx in range(1, 7)]
        self.lock_for_task("MOVE_JOINT", "-")
        self.write_log("관절 이동 시작")
        self.write_developer_log(f"manual joint move target={joints}")
        self.runner.send_manual_command({"mode": "move-joint", "joints": joints})

    # 현재 TCP 베이스 좌표를 요청하여 직교 좌표 입력 위젯에 표시한다.
    def run_read_base(self) -> None:
        if self.logic.is_busy:
            return
        self.lock_for_task("READ_BASE", "-")
        self.write_log("현재 TCP 위치 읽기")
        self.runner.send_manual_command({"mode": "read-base"})

    # 화면에 입력된 직교 좌표로 TCP의 선형 이동을 요청한다.
    def run_move_base(self) -> None:
        if self.logic.is_busy:
            return
        pose = [getattr(self, f"spin_{name}").value() for name in ("x", "y", "z", "rx", "ry", "rz")]
        self.lock_for_task("MOVE_BASE", "-")
        self.write_log("TCP 이동 시작")
        self.write_developer_log(f"manual base move target={pose}")
        self.runner.send_manual_command({"mode": "move-base", "pose": pose})

    # 보유 툴을 지정 거치대에 먼저 반환한 뒤 안전하게 HOME 자세로 이동한다.
    def run_home(self) -> None:
        if self.logic.is_busy:
            self.write_log("작업 중에는 HOME을 실행하지 않음")
            return
        if not self.logic.ros_connected:
            self.write_log("ROS 연결 전에는 HOME을 실행하지 않음")
            return
        if self.logic.current_tool_state == config.TOOL_STATE_PRESS:
            self.pending_task = ("home", "-")
            self.lock_for_task("PRESS_PLATE_PUTDOWN", "-")
            self.write_log("HOME 전 누름판을 지정 거치대에 반환")
            self.start_tool_node("press", "putdown")
            return
        if self.logic.current_tool_state == config.TOOL_STATE_SHOVEL:
            self.pending_task = ("home", "-")
            self.lock_for_task("SHOVEL_PUTDOWN", "-")
            self.write_log("HOME 전 삽을 지정 거치대에 반환")
            self.start_tool_node("shovel", "putdown")
            return
        if self.logic.current_tool_state == config.TOOL_STATE_UNKNOWN:
            self.write_log("HOME 이동 전 현재 툴 확인이 필요합니다")
            self.confirm_unknown_tool_and_run_home()
            return
        self._start_home_task()

    # UNKNOWN 상태에서는 작업자가 실물을 확인한 결과에 따라 안전한 HOME 경로를 선택한다.
    def confirm_unknown_tool_and_run_home(self) -> None:
        message_box = QtWidgets.QMessageBox(self)
        message_box.setIcon(QtWidgets.QMessageBox.Warning)
        message_box.setWindowTitle("현재 툴 확인")
        message_box.setText(
            "현재 툴 상태를 확인할 수 없습니다.\n"
            "로봇에 실제로 장착된 툴을 확인한 후 선택하세요."
        )
        press_button = message_box.addButton(
            "누름판 장착",
            QtWidgets.QMessageBox.ActionRole,
        )
        shovel_button = message_box.addButton(
            "삽 장착",
            QtWidgets.QMessageBox.ActionRole,
        )
        none_button = message_box.addButton(
            "툴 없음",
            QtWidgets.QMessageBox.ActionRole,
        )
        cancel_button = message_box.addButton(QtWidgets.QMessageBox.Cancel)
        message_box.setDefaultButton(cancel_button)
        message_box.exec_()

        clicked_button = message_box.clickedButton()
        if clicked_button is press_button:
            confirmed_state = config.TOOL_STATE_PRESS
            confirmed_text = "누름판 장착"
        elif clicked_button is shovel_button:
            confirmed_state = config.TOOL_STATE_SHOVEL
            confirmed_text = "삽 장착"
        elif clicked_button is none_button:
            confirmed_state = config.TOOL_STATE_NONE
            confirmed_text = "툴 없음"
        else:
            self.write_log("툴 확인을 취소하여 HOME 이동을 시작하지 않았습니다")
            return

        self.logic.set_tool_state(confirmed_state)
        self.write_log(f"작업자 툴 상태 확인: {confirmed_text}")
        # 확정된 상태를 기존 HOME 흐름에 다시 전달하여 필요하면 툴부터 반환한다.
        self.run_home()

    # 툴이 없는 상태가 확인된 뒤 실제 HOME 이동 노드를 시작한다.
    def _start_home_task(self) -> None:
        self.lock_for_task("HOME", "-")
        self.write_log("HOME 실행")
        self.runner.start_node(config.HOME_NODE)

    # 버튼 상태와 별개로 실행 직전에 현재 툴을 다시 확인해 잘못된 위치 이동을 막는다.
    # UNKNOWN은 작업자가 현장에서 실제 툴을 확인해 조작할 수 있도록 허용한다.
    def validate_manual_tool_command(
        self,
        action_name: str,
        allowed_states: set[str],
    ) -> bool:
        current_state = self.logic.current_tool_state
        if current_state in allowed_states or current_state == config.TOOL_STATE_UNKNOWN:
            return True

        current_tool_name = {
            config.TOOL_STATE_NONE: "장착된 툴 없음",
            config.TOOL_STATE_PRESS: "누름판 장착",
            config.TOOL_STATE_SHOVEL: "삽 장착",
        }.get(current_state, "툴 전환 중")
        message = (
            f"{action_name} 작업을 실행할 수 없습니다.\n"
            f"현재 툴 상태: {current_tool_name}"
        )
        self.write_log(f"{action_name} 실행 취소 - 현재 툴 상태가 맞지 않습니다")
        QtWidgets.QMessageBox.warning(self, "툴 상태 확인", message)
        self.refresh_button_state()
        return False

    # 작업자 요청에 따라 누름판 장착 작업을 실행한다.
    def run_press_pickup(self) -> None:
        if self.logic.is_busy:
            return
        if not self.validate_manual_tool_command(
            "누름판 집기",
            {config.TOOL_STATE_NONE},
        ):
            return
        self.lock_for_task("PRESS_PLATE_PICKUP", "-")
        self.write_log("누름판 집기 실행")
        self.start_tool_node("press", "pickup")

    # 작업자 요청에 따라 누름판 반환 작업을 실행한다.
    def run_press_putdown(self) -> None:
        if self.logic.is_busy:
            return
        if not self.validate_manual_tool_command(
            "누름판 놓기",
            {config.TOOL_STATE_PRESS},
        ):
            return
        self.lock_for_task("PRESS_PLATE_PUTDOWN", "-")
        self.write_log("누름판 놓기 실행")
        self.start_tool_node("press", "putdown")

    # 작업자 요청에 따라 삽 장착 작업을 실행한다.
    def run_shovel_pickup(self) -> None:
        if self.logic.is_busy:
            return
        if not self.validate_manual_tool_command(
            "삽 집기",
            {config.TOOL_STATE_NONE},
        ):
            return
        self.lock_for_task("SHOVEL_PICKUP", "-")
        self.write_log("삽 집기 실행")
        self.start_tool_node("shovel", "pickup")

    # 작업자 요청에 따라 삽 반환 작업을 실행한다.
    def run_shovel_putdown(self) -> None:
        if self.logic.is_busy:
            return
        if not self.validate_manual_tool_command(
            "삽 놓기",
            {config.TOOL_STATE_SHOVEL},
        ):
            return
        self.lock_for_task("SHOVEL_PUTDOWN", "-")
        self.write_log("삽 놓기 실행")
        self.start_tool_node("shovel", "putdown")

    # 로봇을 움직이지 않고 대시보드 시스템과 기능 버튼을 활성화한다.
    def start_system(self) -> None:
        # 감시기가 joint_states 연속 수신으로 실제 연결을 확인한 뒤에만 시작한다.
        if not self.robot_link_available:
            self.write_log("실제 로봇 연결이 확인되지 않아 시스템을 시작하지 않음")
            QtWidgets.QMessageBox.warning(
                self,
                "로봇 연결 오류",
                "joint_states 수신이 확인되지 않았습니다. 로봇 연결을 확인하세요.",
            )
            return
        self.logic.ros_connected = True
        self.logic.system_started = True
        self.set_label("lbl_ros_status", "Connected")
        self.write_log("시스템 시작 - 기능 버튼 활성화")
        self.refresh_button_state()

    # 실행 작업이 없는 상태에서 일반 기능 버튼을 비활성화한다.
    def shutdown_system(self) -> None:
        self.logic.system_started = False
        self.logic.ros_connected = False
        self.set_label("lbl_ros_status", "Disconnected")
        self.write_log("시스템 종료 - 시작 버튼 외 모든 버튼 비활성화")
        self.refresh_button_state()

    # 서보를 유지한 채 현재 이동을 빠르게 멈추고 후속 이동 명령을 차단한다.
    def emergency_stop(self) -> None:
        if self.logic.is_safety_stopped or self.logic.is_emergency_stopped:
            return

        self.resolve_interrupted_tool_state()
        self.safety_stop_state_snapshot = {
            "system_started": self.logic.system_started,
            "tool_state": self.logic.current_tool_state,
            "tray_state": deepcopy(self.logic.tray_state),
        }
        preserved_tool_state = self.logic.current_tool_state
        self.logic.is_safety_stopped = True
        self.logic.requires_home_after_estop = True
        self.pending_task = None
        # 플래그를 먼저 만들어 현재 프로세스가 다음 move로 넘어가는 경쟁 조건을 막는다.
        activate_safety_stop()
        estop_process = self.runner.start_auxiliary_node(config.INTEGRATED_ESTOP_NODE)
        estop_started = estop_process.waitForStarted(1000)
        self.runner.stop_current_process()
        self.logic.current_running_task = ""
        self.logic.current_running_tray = ""
        self.logic.current_process = None
        if estop_started:
            self.write_log("안전정지 실행 - 로봇 이동을 중지했습니다")
            self.write_developer_log("MoveStop(DR_QSTOP) auxiliary node started")
        else:
            self.write_log("안전정지 명령 실행 실패 - 로봇 상태를 확인하세요")
            self.write_developer_log("MoveStop auxiliary node failed to start", level="ERROR")
        self.write_developer_log(
            f"safety stop preserved tool state={preserved_tool_state}",
            level="INFO",
        )
        self.set_label("lbl_robot_status", "안전정지")
        self.logic.is_busy = False
        self.refresh_button_state()
        self.show_emergency_popup()

    # 툴 작업 단계에 따라 안전정지 순간에 확실히 알 수 있는 상태로 보수적으로 정리한다.
    def resolve_interrupted_tool_state(self) -> None:
        tool_state = self.logic.current_tool_state
        resolved_state = {
            config.TOOL_STATE_PRESS_PICKING_APPROACH: config.TOOL_STATE_NONE,
            config.TOOL_STATE_SHOVEL_PICKING_APPROACH: config.TOOL_STATE_NONE,
            config.TOOL_STATE_PRESS_PUTTING_APPROACH: config.TOOL_STATE_PRESS,
            config.TOOL_STATE_SHOVEL_PUTTING_APPROACH: config.TOOL_STATE_SHOVEL,
            config.TOOL_STATE_PRESS_PICKING_GRIP: config.TOOL_STATE_UNKNOWN,
            config.TOOL_STATE_SHOVEL_PICKING_GRIP: config.TOOL_STATE_UNKNOWN,
            config.TOOL_STATE_PRESS_PUTTING_RELEASE: config.TOOL_STATE_UNKNOWN,
            config.TOOL_STATE_SHOVEL_PUTTING_RELEASE: config.TOOL_STATE_UNKNOWN,
        }.get(tool_state, tool_state)
        if resolved_state != tool_state:
            self.logic.set_tool_state(resolved_state)
            self.write_developer_log(
                f"interrupted tool state resolved: {tool_state} -> {resolved_state}",
                level="INFO",
            )
            if resolved_state == config.TOOL_STATE_UNKNOWN:
                self.write_log("툴 상태 확인이 필요합니다")

    # 긴급정지 후 Tool/TCP 설정을 복원하여 HOME 복귀 준비를 수행한다.
    def recover_from_estop(self) -> None:
        if self.logic.is_safety_stopped:
            self.recover_from_safety_stop()
            return
        if not self.logic.is_emergency_stopped or not self.physical_recovery_ready:
            self.write_log("물리 안전복구 확인 전에는 대시보드 복구를 실행하지 않음")
            return

        if self.last_robot_state == config.ROBOT_STATE_SAFE_OFF:
            self.physical_recovery_ready = False
            self.recovery_command_pending = True
            self.recovery_attempt_id += 1
            attempt_id = self.recovery_attempt_id
            self.set_label("lbl_robot_status", "서보 안전복구 요청 중")
            self.write_log("로봇 안전복구 요청 중")
            self.write_developer_log(
                "CONTROL_RESET_SAFE_OFF requested; waiting for STANDBY",
                level="INFO",
            )
            self.refresh_button_state()
            self.connection_monitor.request_safe_off_reset()
            QTimer.singleShot(
                config.ROBOT_RECOVERY_CONFIRM_TIMEOUT_MS,
                lambda token=attempt_id: self.handle_recovery_confirmation_timeout(token),
            )
            return

        if self.last_robot_state != config.ROBOT_STATE_STANDBY:
            self.write_log("현재 로봇 상태에서는 안전복구할 수 없습니다")
            self.write_developer_log(
                f"recovery rejected for robot_state={self.last_robot_state}",
                level="WARNING",
            )
            return

        self.complete_dashboard_recovery()

    # 소프트웨어 안전정지는 서보 복구 없이 논리 상태를 복원하고 HOME만 요구한다.
    def recover_from_safety_stop(self) -> None:
        snapshot = self.safety_stop_state_snapshot or {}
        self.logic.system_started = bool(snapshot.get("system_started", True))
        self.logic.current_tool_state = snapshot.get(
            "tool_state", self.logic.current_tool_state
        )
        saved_trays = snapshot.get("tray_state")
        if saved_trays is not None:
            self.logic.tray_state = deepcopy(saved_trays)
            for tray_label in config.TRAY_LABELS:
                self.update_tray_widgets(tray_label)
        self.logic.save_persistent_state()

        # HOME 노드는 이동해야 하므로 안전복구 시 공유 정지 플래그를 해제한다.
        clear_safety_stop()
        self.logic.is_safety_stopped = False
        self.logic.is_busy = False
        self.logic.current_running_task = ""
        self.logic.current_running_tray = ""
        self.logic.current_process = None
        self.logic.requires_home_after_estop = self.logic.system_started
        self.logic.ros_connected = self.robot_link_available
        self.safety_stop_state_snapshot = None
        self.set_label(
            "lbl_robot_status",
            "초기 위치 이동 필요" if self.logic.system_started else "대기 중",
        )
        self.write_log("안전정지 복구 - 이전 논리 상태 복원, HOME 필요")
        self.refresh_button_state()
        if self.logic.requires_home_after_estop:
            self.show_home_required_warning()

    # 안전복구 후 다른 작업 전에 HOME 복귀가 필요함을 작업자에게 명확히 알린다.
    def show_home_required_warning(self) -> None:
        QtWidgets.QMessageBox.warning(
            self,
            "HOME 이동 필요",
            "안전복구가 완료되었습니다.\n"
            "다른 작업을 시작하기 전에 HOME 버튼을 눌러 로봇을 HOME 위치로 이동하세요.",
        )

    # set_robot_control 서비스 호출 결과를 기록하고 실패 시 안전 잠금을 유지한다.
    def handle_safe_off_reset_finished(self, success: bool, message: str) -> None:
        self.write_developer_log(
            f"SAFE_OFF reset result: success={success}, message={message}",
            level="INFO" if success else "ERROR",
        )
        if success:
            return
        if not self.recovery_command_pending:
            return
        self.recovery_command_pending = False
        self.physical_recovery_ready = self.last_robot_state in config.ROBOT_RECOVERY_READY_STATES
        self.set_label("lbl_robot_status", "서보 안전복구 실패 - 상태를 확인하세요")
        self.write_log("안전복구 실패 - 로봇 상태를 확인하세요")
        self.refresh_button_state()
        QtWidgets.QMessageBox.warning(self, "안전복구 실패", message)

    # 서비스가 성공해도 STANDBY가 확인되지 않으면 이전 UI 상태를 복원하지 않는다.
    def handle_recovery_confirmation_timeout(self, attempt_id: int) -> None:
        if attempt_id != self.recovery_attempt_id or not self.recovery_command_pending:
            return
        self.recovery_command_pending = False
        self.physical_recovery_ready = self.last_robot_state in config.ROBOT_RECOVERY_READY_STATES
        self.set_label("lbl_robot_status", "대기 상태 확인 시간 초과")
        self.write_log("안전복구 시간 초과 - 로봇 상태를 확인하세요")
        self.refresh_button_state()
        QtWidgets.QMessageBox.warning(
            self,
            "안전복구 시간 초과",
            "로봇이 STANDBY 상태로 전환되지 않았습니다.",
        )

    # STANDBY 확인 후에만 저장했던 논리 상태를 복원하고 HOME을 요구한다.
    def complete_dashboard_recovery(self) -> None:

        snapshot = self.estop_state_snapshot or {}
        self.logic.system_started = bool(snapshot.get("system_started", True))
        self.logic.current_tool_state = snapshot.get(
            "tool_state", self.logic.current_tool_state
        )
        saved_trays = snapshot.get("tray_state")
        if saved_trays is not None:
            self.logic.tray_state = deepcopy(saved_trays)
            for tray_label in config.TRAY_LABELS:
                self.update_tray_widgets(tray_label)
        self.logic.save_persistent_state()

        self.logic.is_emergency_stopped = False
        self.logic.requires_home_after_estop = self.logic.system_started
        self.logic.ros_connected = self.robot_link_available
        self.physical_recovery_ready = False
        self.recovery_command_pending = False
        self.estop_state_snapshot = None
        clear_safety_stop()

        if not self.logic.system_started:
            self.set_label("lbl_robot_status", "대기 중")
            self.refresh_button_state()
            return

        preserved_tool_state = self.logic.current_tool_state
        self.lock_for_task("RECOVER_TOOL_TCP", "-")
        self.write_log("로봇 설정 복구 중")
        self.write_developer_log(
            "physical recovery restoring Tool/TCP; "
            f"preserved_tool_state={preserved_tool_state}",
            level="INFO",
        )
        self.runner.send_manual_command({"mode": "recover-config"})

    # 상시 수동 제어 노드 종료 시 참조와 작업 잠금을 정리한다.
    def handle_manual_server_finished(
        self,
        process: QProcess,
        exit_code: int,
        _exit_status,
    ) -> None:
        if self.runner.manual_process is process:
            self.runner.manual_process = None
        self.write_developer_log(
            f"persistent manual server exited: code={exit_code}",
            level="INFO" if exit_code == 0 else "ERROR",
        )
        if self.logic.current_running_task in {
            "READ_JOINT",
            "READ_BASE",
            "MOVE_JOINT",
            "MOVE_BASE",
            "RECOVER_TOOL_TCP",
        }:
            self.unlock_after_task()
        process.deleteLater()

    # 긴급정지 등 보조 노드의 종료 결과를 기록하고 프로세스를 정리한다.
    def handle_auxiliary_process_finished(
        self,
        process: QProcess,
        node_name: str,
        exit_code: int,
        _exit_status,
    ) -> None:
        if exit_code == 0:
            self.write_developer_log(f"auxiliary node completed: {node_name}")
        else:
            self.write_developer_log(
                f"auxiliary node failed: {node_name}, code={exit_code}",
                level="ERROR",
            )
            self.write_log("안전 보조 명령 실행 실패 - 로봇 상태를 확인하세요")
        process.deleteLater()

    # 자식 ROS 원문은 개발자 파일에만 저장하고 UI에는 해석한 결과만 표시한다.
    def handle_process_output(self, process: QProcess) -> None:
        raw = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in raw.splitlines():
            if not line.strip():
                continue
            self.write_developer_log(line, level="ROS")
            self._consume_process_line(line)

    # ROS 노드의 정형 로그 한 줄을 대시보드 상태와 입력값으로 변환한다.
    def _consume_process_line(self, line: str) -> None:
        tray_label = self.logic.current_running_tray

        # 순응제어로 표면을 찾은 순간의 Z값을 작업자가 확인할 수 있게 표시한다.
        # 예: A_corner_1 contact detected z=71.470
        #     A_corner_1 flatten contact z=69.032
        contact_match = re.search(
            r"\b(?P<tray>[A-D])_corner_(?P<point>[1-4])\s+"
            r"(?:(?P<flatten>flatten)\s+)?contact(?:\s+detected)?\s+"
            r"z=(?P<z>-?\d+(?:\.\d+)?)",
            line,
        )
        if contact_match:
            operation = "평탄화" if contact_match.group("flatten") else "측정"
            z_value = float(contact_match.group("z"))
            self.write_log(
                f"{contact_match.group('tray')} 트레이 {operation} 지점 "
                f"{contact_match.group('point')} Z: {z_value:.3f} mm"
            )
            return

        measurement_failure = re.search(
            r"\b(?P<tray>[A-D])_corner_(?P<point>[1-4])\s+"
            r"contact failed, limit reached z=(?P<z>-?\d+(?:\.\d+)?)",
            line,
        )
        if measurement_failure:
            z_value = float(measurement_failure.group("z"))
            self.write_log(
                f"{measurement_failure.group('tray')} 트레이 측정 지점 "
                f"{measurement_failure.group('point')}: 접촉 실패 "
                f"(한계 Z: {z_value:.3f} mm)"
            )
            return

        flatten_failure = re.search(
            r"\b(?P<tray>[A-D])_corner_(?P<point>[1-4])\s+"
            r"flatten contact failed",
            line,
        )
        if flatten_failure:
            self.write_log(
                f"{flatten_failure.group('tray')} 트레이 평탄화 지점 "
                f"{flatten_failure.group('point')}: 접촉 실패"
            )
            return

        if "soil_state=SOIL_LOW" in line:
            self.logic.set_soil_status(tray_label, config.SOIL_STATUS_LOW)
            self.update_tray_widgets(tray_label)
            self.write_log(f"{tray_label} 트레이 측정 결과: 흙 부족")
        elif "soil_state=SOIL_OK" in line:
            self.logic.set_soil_status(tray_label, config.SOIL_STATUS_OK)
            self.update_tray_widgets(tray_label)
            self.write_log(f"{tray_label} 트레이 측정 결과: 정상")
        elif "soil_state=SOIL_HIGH" in line:
            self.logic.set_soil_status(tray_label, config.SOIL_STATUS_HIGH)
            self.update_tray_widgets(tray_label)
            self.write_log(f"{tray_label} 트레이 측정 결과: 흙 많음")
        elif "soil_state=SKIPPED_DUE_TO_ROCK" in line:
            self.logic.set_soil_status(tray_label, config.SOIL_STATUS_OBSTACLE)
            self.update_tray_widgets(tray_label)
            self.write_log(f"{tray_label} 트레이 측정 결과: 장애물 감지")
        elif "measurement_result_json=" in line:
            payload = line.split("measurement_result_json=", 1)[1]
            self.logic.set_last_measurement(tray_label, json.loads(payload))
        elif "joint_position_json=" in line:
            payload = json.loads(line.split("joint_position_json=", 1)[1])
            self._set_joint_spin_values(payload)
        elif "base_pose_json=" in line:
            payload = json.loads(line.split("base_pose_json=", 1)[1])
            self._set_base_spin_values(payload)
        elif "manual_command_result_json=" in line:
            payload = json.loads(line.split("manual_command_result_json=", 1)[1])
            mode = payload.get("mode", "unknown")
            if self.logic.is_emergency_stopped or self.logic.is_safety_stopped:
                self.write_developer_log(
                    f"manual result ignored while stopped: mode={mode}",
                    level="WARNING",
                )
                return
            if payload.get("success"):
                user_message = {
                    "read-joint": "현재 관절값 읽기 완료",
                    "read-base": "현재 TCP 위치 읽기 완료",
                    "move-joint": "관절 이동 완료",
                    "move-base": "TCP 이동 완료",
                    "recover-config": "안전복구 설정 완료",
                }.get(mode, "수동 명령 완료")
                self.write_log(user_message)
                if mode == "recover-config":
                    self.set_label("lbl_robot_status", "안전복구 완료")
            else:
                error = payload.get("error", "unknown error")
                self.write_log("수동 명령 실패 - 로봇 상태와 연결을 확인하세요")
                self.write_developer_log(
                    f"manual command failed: mode={mode}, error={error}",
                    level="ERROR",
                )
                self.logic.last_error_message = str(error)
            self.unlock_after_task()
            if (
                payload.get("success")
                and mode == "recover-config"
                and self.logic.requires_home_after_estop
            ):
                self.show_home_required_warning()
        elif "tool_phase=PRESS_PICKING_APPROACH" in line:
            self.logic.set_tool_state(config.TOOL_STATE_PRESS_PICKING_APPROACH)
        elif "tool_phase=PRESS_PICKING_GRIP" in line:
            self.logic.set_tool_state(config.TOOL_STATE_PRESS_PICKING_GRIP)
        elif "tool_phase=PRESS_PLATE_HELD" in line:
            self.logic.set_tool_state(config.TOOL_STATE_PRESS)
        elif "tool_phase=PRESS_PUTTING_APPROACH" in line:
            self.logic.set_tool_state(config.TOOL_STATE_PRESS_PUTTING_APPROACH)
        elif "tool_phase=PRESS_PUTTING_RELEASE" in line:
            self.logic.set_tool_state(config.TOOL_STATE_PRESS_PUTTING_RELEASE)
        elif "tool_phase=PRESS_RELEASED" in line:
            self.logic.set_tool_state(config.TOOL_STATE_NONE)
        elif "tool_phase=SHOVEL_PICKING_APPROACH" in line:
            self.logic.set_tool_state(config.TOOL_STATE_SHOVEL_PICKING_APPROACH)
        elif "tool_phase=SHOVEL_PICKING_GRIP" in line:
            self.logic.set_tool_state(config.TOOL_STATE_SHOVEL_PICKING_GRIP)
        elif "tool_phase=SHOVEL_HELD" in line:
            self.logic.set_tool_state(config.TOOL_STATE_SHOVEL)
        elif "tool_phase=SHOVEL_PUTTING_APPROACH" in line:
            self.logic.set_tool_state(config.TOOL_STATE_SHOVEL_PUTTING_APPROACH)
        elif "tool_phase=SHOVEL_PUTTING_RELEASE" in line:
            self.logic.set_tool_state(config.TOOL_STATE_SHOVEL_PUTTING_RELEASE)
        elif "tool_phase=SHOVEL_RELEASED" in line:
            self.logic.set_tool_state(config.TOOL_STATE_NONE)

    # 읽어 온 관절값을 여섯 개의 관절 입력 스핀 박스에 채운다.
    def _set_joint_spin_values(self, joint_values: list[float]) -> None:
        for idx, value in enumerate(joint_values[:6], start=1):
            spin = getattr(self, f"spin_j{idx}", None)
            if spin is not None:
                spin.setValue(float(value))

    # 읽어 온 TCP 자세값을 위치·회전 입력 스핀 박스에 채운다.
    def _set_base_spin_values(self, pose_values: list[float]) -> None:
        for name, value in zip(("x", "y", "z", "rx", "ry", "rz"), pose_values[:6]):
            spin = getattr(self, f"spin_{name}", None)
            if spin is not None:
                spin.setValue(float(value))

    # 전면 ROS 작업의 결과를 반영하고 예약된 후속 툴·트레이 작업을 이어간다.
    def handle_process_finished(
        self,
        process: QProcess,
        node_name: str,
        exit_code: int,
        exit_status,
    ) -> None:
        if self.runner.process is process:
            self.runner.process = None
        self.logic.current_process = None
        completed_task = self.logic.current_running_task
        task_label = self.user_task_name(completed_task)
        self.write_developer_log(
            f"foreground process finished: node={node_name}, task={completed_task}, "
            f"exit_code={exit_code}, exit_status={exit_status}",
            level="INFO" if exit_code == 0 else "ERROR",
        )

        if self.logic.is_emergency_stopped:
            self.refresh_button_state()
            return

        tray_label = self.logic.current_running_tray
        if exit_code == 0:
            self.write_log(f"{task_label} 완료")
            if node_name == config.INTEGRATED_TOOL_NODE and completed_task == "PRESS_PLATE_PICKUP":
                self.logic.set_tool_state(config.TOOL_STATE_PRESS)
                pending_task = self.pending_task
                self.pending_task = None
                if pending_task == ("measure", tray_label):
                    self.run_after_inter_node_delay(
                        lambda tray=tray_label: self._start_measure_task(tray),
                        reason="press pickup 완료",
                        delay_ms=config.PRESS_PICKUP_HANDOFF_DELAY_MS,
                    )
                    return
                if pending_task == ("flatten", tray_label):
                    self.run_after_inter_node_delay(
                        lambda tray=tray_label: self._start_flatten_task(tray),
                        reason="press pickup 완료",
                        delay_ms=config.PRESS_PICKUP_HANDOFF_DELAY_MS,
                    )
                    return
            if node_name == config.INTEGRATED_TOOL_NODE and completed_task == "PRESS_PLATE_PUTDOWN":
                self.logic.set_tool_state(config.TOOL_STATE_NONE)
                pending_task = self.pending_task
                self.pending_task = None
                if pending_task == ("home", "-"):
                    self.run_after_inter_node_delay(
                        self._start_home_task,
                        reason="누름판 반환 완료",
                        delay_ms=config.PRESS_PUTDOWN_HANDOFF_DELAY_MS,
                    )
                    return
                if pending_task == ("next", tray_label):
                    self.run_after_inter_node_delay(
                        lambda tray=tray_label: self._start_next_step_task(
                            tray,
                            skip_initial_home=True,
                        ),
                        reason="press putdown 완료",
                        delay_ms=config.PRESS_PUTDOWN_HANDOFF_DELAY_MS,
                    )
                    return
            if node_name == config.INTEGRATED_TOOL_NODE and completed_task == "SHOVEL_PICKUP":
                self.logic.set_tool_state(config.TOOL_STATE_SHOVEL)
            if node_name == config.INTEGRATED_TOOL_NODE and completed_task == "SHOVEL_PUTDOWN":
                self.logic.set_tool_state(config.TOOL_STATE_NONE)
                pending_task = self.pending_task
                self.pending_task = None
                if pending_task == ("home", "-"):
                    self.run_after_inter_node_delay(
                        self._start_home_task,
                        reason="삽 반환 완료",
                    )
                    return
                if pending_task == ("flatten_press_pickup", tray_label):
                    self.pending_task = ("flatten", tray_label)
                    # 삽 반환 후 안정화가 끝나면 평탄화용 누름판 장착을 시작한다.
                    def _start_press_pickup_after_shovel_putdown(tray=tray_label):
                        self.lock_for_task("PRESS_PLATE_PICKUP", tray)
                        self.write_log("평탄화용 누름판을 장착합니다")
                        self.start_tool_node("press", "pickup")

                    self.run_after_inter_node_delay(
                        _start_press_pickup_after_shovel_putdown,
                        reason="shovel putdown 완료",
                    )
                    return
                if pending_task == ("measure_press_pickup", tray_label):
                    self.pending_task = ("measure", tray_label)
                    # 삽을 거치한 뒤 토양 측정용 누름판 장착을 이어서 실행한다.
                    def _start_press_pickup_for_measure(tray=tray_label):
                        self.lock_for_task("PRESS_PLATE_PICKUP", tray)
                        self.write_log("토양 측정용 누름판을 장착합니다")
                        self.start_tool_node("press", "pickup")

                    self.run_after_inter_node_delay(
                        _start_press_pickup_for_measure,
                        reason="shovel putdown 완료",
                    )
                    return
            if node_name == config.INTEGRATED_NEXT_STEP_NODE and tray_label in self.logic.tray_state:
                if self.logic.current_running_task in (
                    config.NEXT_ACTION_ADD_SOIL,
                    config.NEXT_ACTION_REMOVE_SOIL,
                    config.NEXT_ACTION_REMOVE_ROCK,
                ):
                    self.logic.set_tool_state(config.TOOL_STATE_NONE)
                    self.logic.mark_corrective_action_completed(tray_label)
                    self.update_tray_widgets(tray_label)
                elif self.logic.current_running_task == config.NEXT_ACTION_FLATTEN:
                    self.logic.set_tool_state(config.TOOL_STATE_NONE)
                    self.logic.mark_flatten_completed(tray_label)
                    self.update_tray_widgets(tray_label)
                elif self.logic.current_running_task == config.NEXT_ACTION_PLANT:
                    self.logic.set_tool_state(config.TOOL_STATE_NONE)
                    self.logic.mark_plant_completed(tray_label)
                    self.update_tray_widgets(tray_label)
            if node_name == config.INTEGRATED_FLATTEN_NODE and tray_label in self.logic.tray_state:
                self.logic.set_tool_state(config.TOOL_STATE_PRESS)
                self.logic.mark_flatten_completed(tray_label)
                self.update_tray_widgets(tray_label)
            if node_name == config.HOME_NODE:
                # HOME 흐름은 보유 툴을 먼저 반환하므로 성공 시 툴 없음이 확정된다.
                self.logic.set_tool_state(config.TOOL_STATE_NONE)
                self.logic.requires_home_after_estop = False
        else:
            self.pending_task = None
            # 실패 시에도 접근/그립/해제 단계에 따라 안전하게 툴 상태를 확정한다.
            self.resolve_interrupted_tool_state()
            self.logic.set_error(tray_label, f"{node_name} failed with code {exit_code}")
            self.write_log(f"{task_label} 실패 - 로봇 상태와 연결을 확인하세요")

        self.unlock_after_task()
