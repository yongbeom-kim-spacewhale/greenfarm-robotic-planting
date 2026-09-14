"""통합 대시보드에서 ROS 노드 프로세스를 실행한다."""

from __future__ import annotations

from pathlib import Path
import json
import shlex

from PyQt5.QtCore import QProcess

from robot_tests.integrated import config


class RosNodeRunner:
    """대시보드 작업과 ROS 2 명령행 노드 프로세스를 연결한다.

    기본 ``process`` 슬롯은 물리 로봇 작업을 순차 실행한다. 수동 제어는
    지속 실행되는 표준 입력 서버를 사용하며, 긴급 보조 기능은 기본 작업이
    실행 중일 때도 동작할 수 있도록 독립 프로세스를 사용한다.
    """

    # ROS 프로세스 실행 결과를 받을 대시보드와 프로세스 참조를 초기화한다.
    def __init__(self, owner) -> None:
        self.owner = owner
        self.process: QProcess | None = None
        self.manual_process: QProcess | None = None

    # 수동 제어 명령을 지속적으로 받는 ROS 노드를 시작하거나 재사용한다.
    def start_manual_server(self) -> QProcess:
        if self.manual_process is not None and self.manual_process.state() != QProcess.NotRunning:
            return self.manual_process

        process = QProcess(self.owner)
        process.setProgram("/bin/bash")
        setup_path = Path(config.WORKSPACE_SETUP_PATH)
        shell_cmd = (
            f"source '{setup_path}' && ros2 run robot_tests "
            f"{config.INTEGRATED_MANUAL_NODE} -- --mode server"
        )
        process.setArguments(["-lc", shell_cmd])
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: self.owner.handle_process_output(process)
        )
        process.finished.connect(
            lambda exit_code, exit_status: self.owner.handle_manual_server_finished(
                process, exit_code, exit_status
            )
        )
        process.start()
        self.manual_process = process
        return process

    # 수동 제어 서버에 관절 또는 베이스 제어 명령을 JSON으로 전송한다.
    def send_manual_command(self, command: dict) -> None:
        process = self.start_manual_server()
        if not process.waitForStarted(3000):
            raise RuntimeError("Persistent manual control node failed to start")
        payload = json.dumps(command, ensure_ascii=False, separators=(",", ":")) + "\n"
        process.write(payload.encode("utf-8"))

    # 대시보드가 추적할 단일 전면 ROS 작업 노드를 실행한다.
    def start_node(
        self,
        node_name: str,
        *,
        node_args: list[str] | None = None,
        input_text: str = "",
    ) -> QProcess:
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            raise RuntimeError("Another ROS node process is already running")

        process = QProcess(self.owner)
        process.setProgram("/bin/bash")
        setup_path = Path(config.WORKSPACE_SETUP_PATH)
        shell_cmd = f"source '{setup_path}' && ros2 run robot_tests {node_name}"
        if node_args:
            escaped_args = " ".join(shlex.quote(arg) for arg in node_args)
            shell_cmd = f"{shell_cmd} -- {escaped_args}"
        process.setArguments(["-lc", shell_cmd])
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: self.owner.handle_process_output(process)
        )
        process.finished.connect(
            lambda exit_code, exit_status: self.owner.handle_process_finished(
                process, node_name, exit_code, exit_status
            )
        )
        process.start()

        self.process = process
        self.owner.logic.current_process = process

        if input_text:
            process.waitForStarted(3000)
            process.write(input_text.encode("utf-8"))

        return process

    # 대시보드 작업 추적과 분리하여 독립적인 ROS 노드를 실행한다.
    def start_detached_node(self, node_name: str, *, node_args: list[str] | None = None) -> bool:
        setup_path = Path(config.WORKSPACE_SETUP_PATH)
        shell_cmd = f"source '{setup_path}' && ros2 run robot_tests {node_name}"
        if node_args:
            escaped_args = " ".join(shlex.quote(arg) for arg in node_args)
            shell_cmd = f"{shell_cmd} -- {escaped_args}"
        return QProcess.startDetached("/bin/bash", ["-lc", shell_cmd])

    # 현재 주 작업을 유지한 채 긴급정지 등의 보조 ROS 노드를 실행한다.
    def start_auxiliary_node(
        self,
        node_name: str,
        *,
        node_args: list[str] | None = None,
    ) -> QProcess:
        process = QProcess(self.owner)
        process.setProgram("/bin/bash")
        setup_path = Path(config.WORKSPACE_SETUP_PATH)
        shell_cmd = f"source '{setup_path}' && ros2 run robot_tests {node_name}"
        if node_args:
            escaped_args = " ".join(shlex.quote(arg) for arg in node_args)
            shell_cmd = f"{shell_cmd} -- {escaped_args}"
        process.setArguments(["-lc", shell_cmd])
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: self.owner.handle_process_output(process)
        )
        process.finished.connect(
            lambda exit_code, exit_status: self.owner.handle_auxiliary_process_finished(
                process, node_name, exit_code, exit_status
            )
        )
        process.start()
        return process

    # 실행 중인 전면 작업 프로세스를 종료하고 관련 참조를 정리한다.
    def stop_current_process(self, *, suppress_finished: bool = True) -> None:
        if self.process is None:
            return
        process = self.process
        if self.process.state() != QProcess.NotRunning:
            if suppress_finished:
                try:
                    process.finished.disconnect()
                except TypeError:
                    pass
            process.kill()
            process.waitForFinished(2000)
        self.process = None
        self.owner.logic.current_process = None
