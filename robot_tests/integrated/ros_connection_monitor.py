"""ROS 토픽을 이용해 실제 로봇 연결 상태를 감시한다."""

from __future__ import annotations

from threading import Event
from time import monotonic

import rclpy
from dsr_msgs2.msg import RobotDisconnection, RobotError
from dsr_msgs2.srv import GetRobotState, SetRobotControl
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class RobotConnectionMonitor(QThread):
    """ROS executor를 UI와 분리하고 연결 상태 변경을 Qt signal로 전달한다."""

    # 작업 스레드에서 Qt 위젯을 직접 수정하지 않고 메인 UI 스레드로 결과를 보낸다.
    connection_changed = pyqtSignal(bool, str)
    robot_error_received = pyqtSignal(int, int, int, str)
    robot_state_changed = pyqtSignal(int)
    safe_off_reset_finished = pyqtSignal(bool, str)

    def __init__(
        self,
        parent=None,
        *,
        joint_state_topic: str,
        disconnection_topic: str,
        error_topic: str,
        robot_state_service: str,
        robot_control_service: str,
        heartbeat_timeout_seconds: float,
        reconnect_message_count: int,
        state_poll_interval_seconds: float,
        state_request_timeout_seconds: float,
        safe_off_reset_control: int,
    ) -> None:
        super().__init__(parent)
        self.joint_state_topic = joint_state_topic
        self.disconnection_topic = disconnection_topic
        self.error_topic = error_topic
        self.robot_state_service = robot_state_service
        self.robot_control_service = robot_control_service
        self.heartbeat_timeout_seconds = heartbeat_timeout_seconds
        self.reconnect_message_count = reconnect_message_count
        self.state_poll_interval_seconds = state_poll_interval_seconds
        self.state_request_timeout_seconds = state_request_timeout_seconds
        self.safe_off_reset_control = safe_off_reset_control
        self._stop_event = Event()
        self._last_joint_state_time: float | None = None
        self._consecutive_joint_states = 0
        self._connected = False
        self._last_robot_state: int | None = None
        self._state_request_future = None
        self._state_request_started_at = 0.0
        self._next_state_request_at = 0.0
        self._safe_off_reset_requested = Event()
        self._safe_off_reset_future = None
        self._safe_off_reset_started_at = 0.0

    def stop(self) -> None:
        """감시 루프 종료를 요청한다."""
        self._stop_event.set()

    def request_safe_off_reset(self) -> None:
        """UI 스레드에서 SAFE_OFF 복구 요청을 thread-safe하게 전달한다."""
        self._safe_off_reset_requested.set()

    def run(self) -> None:
        """전용 ROS context에서 구독과 heartbeat 감시를 실행한다."""
        context = Context()
        node: Node | None = None
        executor: SingleThreadedExecutor | None = None
        try:
            # 대시보드의 Qt 이벤트 루프와 ROS spin이 서로 막지 않도록 이 스레드만의
            # context와 executor를 만든다. 둘은 반드시 같은 context를 사용해야 한다.
            rclpy.init(context=context)
            node = Node("integrated_dashboard_connection_monitor", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)

            # joint_states는 정상 연결과 재연결을 확인하는 heartbeat로 사용한다.
            node.create_subscription(
                JointState,
                self.joint_state_topic,
                self._handle_joint_state,
                qos_profile_sensor_data,
            )
            # 이 토픽은 로봇 연결이 실제로 끊긴 순간 dsr_controller2가 발행한다.
            node.create_subscription(
                RobotDisconnection,
                self.disconnection_topic,
                self._handle_disconnection,
                10,
            )
            # 안전 컨트롤러 오류의 group/code/message를 대시보드 로그로 전달한다.
            node.create_subscription(
                RobotError,
                self.error_topic,
                self._handle_robot_error,
                10,
            )
            state_client = node.create_client(GetRobotState, self.robot_state_service)
            control_client = node.create_client(SetRobotControl, self.robot_control_service)

            while not self._stop_event.is_set():
                executor.spin_once(timeout_sec=0.2)
                self._check_heartbeat(node)
                self._poll_robot_state(state_client)
                self._process_safe_off_reset(control_client)
        except Exception as exc:
            # 초기 상태도 False이므로 일반 상태 전환 함수로는 시작 오류가 전달되지 않는다.
            self.connection_changed.emit(False, f"연결 감시 오류: {exc}")
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)

    def _handle_joint_state(self, _message: JointState) -> None:
        self._last_joint_state_time = monotonic()
        self._consecutive_joint_states += 1
        # 한 번의 우연한 수신으로 재연결하지 않고 지정 횟수만큼 연속 확인한다.
        if self._consecutive_joint_states >= self.reconnect_message_count:
            self._set_connected(True, "joint_states 연속 수신")

    def _handle_disconnection(self, _message: RobotDisconnection) -> None:
        # 전용 연결 해제 이벤트는 heartbeat timeout을 기다리지 않고 즉시 반영한다.
        self._last_joint_state_time = None
        self._consecutive_joint_states = 0
        self._set_connected(False, "robot_disconnection 이벤트 수신")

    def _handle_robot_error(self, message: RobotError) -> None:
        details = " | ".join(
            text for text in (message.msg1, message.msg2, message.msg3) if text
        )
        self.robot_error_received.emit(
            int(message.level), int(message.group), int(message.code), details
        )

    def _poll_robot_state(self, state_client) -> None:
        """CLI 프로세스 없이 비동기 서비스로 안전 상태 변화를 확인한다."""
        now = monotonic()
        if self._state_request_future is not None:
            if self._state_request_future.done():
                self._state_request_future = None
            elif now - self._state_request_started_at > self.state_request_timeout_seconds:
                self._state_request_future.cancel()
                self._state_request_future = None
            else:
                return

        if now < self._next_state_request_at or not state_client.service_is_ready():
            return

        self._next_state_request_at = now + self.state_poll_interval_seconds
        self._state_request_started_at = now
        future = state_client.call_async(GetRobotState.Request())
        future.add_done_callback(self._handle_robot_state_response)
        self._state_request_future = future

    def _handle_robot_state_response(self, future) -> None:
        try:
            response = future.result()
        except Exception:
            return
        if response is None or not response.success:
            return
        robot_state = int(response.robot_state)
        if robot_state == self._last_robot_state:
            return
        self._last_robot_state = robot_state
        self.robot_state_changed.emit(robot_state)

    def _process_safe_off_reset(self, control_client) -> None:
        """감시 스레드에서만 set_robot_control 클라이언트를 사용한다."""
        now = monotonic()
        if self._safe_off_reset_future is not None:
            if self._safe_off_reset_future.done():
                self._safe_off_reset_future = None
            elif now - self._safe_off_reset_started_at > self.state_request_timeout_seconds:
                self._safe_off_reset_future.cancel()
                self._safe_off_reset_future = None
                self.safe_off_reset_finished.emit(False, "서비스 응답 시간 초과")
            return

        if not self._safe_off_reset_requested.is_set():
            return
        self._safe_off_reset_requested.clear()
        if not control_client.service_is_ready():
            self.safe_off_reset_finished.emit(False, "set_robot_control 서비스 사용 불가")
            return

        request = SetRobotControl.Request()
        request.robot_control = self.safe_off_reset_control
        future = control_client.call_async(request)
        future.add_done_callback(self._handle_safe_off_reset_response)
        self._safe_off_reset_future = future
        self._safe_off_reset_started_at = now

    def _handle_safe_off_reset_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.safe_off_reset_finished.emit(False, str(exc))
            return
        success = bool(response is not None and response.success)
        self.safe_off_reset_finished.emit(
            success,
            "CONTROL_RESET_SAFE_OFF 요청 성공" if success else "로봇이 복구 요청을 거부함",
        )

    def _check_heartbeat(self, node: Node) -> None:
        if not self._connected or self._last_joint_state_time is None:
            return
        if monotonic() - self._last_joint_state_time <= self.heartbeat_timeout_seconds:
            return

        # 순간적인 메시지 지연만으로 연결을 끊지 않고 publisher 소멸까지 함께 확인한다.
        if node.count_publishers(self.joint_state_topic) == 0:
            self._consecutive_joint_states = 0
            self._set_connected(False, "joint_states heartbeat 및 publisher 소멸")

    def _set_connected(self, connected: bool, reason: str) -> None:
        # 같은 상태의 signal 반복과 경고창 중복 표시를 방지한다.
        if self._connected == connected:
            return
        self._connected = connected
        self.connection_changed.emit(connected, reason)
