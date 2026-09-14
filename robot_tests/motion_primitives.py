# 로봇 테스트 노드들이 공통으로 사용하는 이동/그리퍼 보조 함수 모음이다.
"""Shared motion/gripper primitives for robot test nodes."""

from __future__ import annotations

import time

import rclpy

from robot_tests.robot_motion_data import GRIPPER_HOME_PREPARE, GRIPPER_PICK_CLOSE, HOME_JOINT
from robot_tests.safety_state import ensure_motion_allowed

DEFAULT_VELOCITY = 60.0
DEFAULT_ACCELERATION = 60.0
DEFAULT_WAIT = 0.5
DEFAULT_HOME_Z_OFFSET = 120.0
HOME_MOVE_VELOCITY = 75.0
HOME_MOVE_ACCELERATION = 70.0
DIGITAL_OUTPUT_TIMEOUT_SEC = 5.0
DIGITAL_OUTPUT_MAX_ATTEMPTS = 3
DIGITAL_OUTPUT_RETRY_DELAY_SEC = 0.3
HOME_JOINT_TOLERANCE_DEG = 1.0
CURRENT_JOINT_TIMEOUT_SEC = 5.0


def normalize_pose(raw_pose: object) -> list[float]:
    """Doosan pose 반환형을 6개 float 리스트로 정규화."""
    pose = raw_pose

    if isinstance(raw_pose, tuple):
        if not raw_pose:
            raise ValueError("Current pose tuple is empty")
        pose = raw_pose[0]

    if hasattr(pose, "tolist"):
        pose = pose.tolist()

    if not isinstance(pose, (list, tuple)):
        raise TypeError(f"Unsupported pose type: {type(pose).__name__}")

    pose_list = list(pose)
    if len(pose_list) < 6:
        raise ValueError(f"Current pose has insufficient length: {len(pose_list)}")

    return [float(value) for value in pose_list[:6]]


def pose(values: list[float]) -> list[float]:
    """Validate a six-element robot joint or Cartesian pose before SDK use."""
    if len(values) != 6:
        raise ValueError(f"Pose must have 6 elements, got {len(values)}")
    return [float(value) for value in values]


def is_home_joint(current_joint: object) -> bool:
    """Check whether all six joints are within the configured HOME tolerance."""
    joint_values = normalize_pose(current_joint)
    return all(
        abs(current - target) <= HOME_JOINT_TOLERANCE_DEG
        for current, target in zip(joint_values, HOME_JOINT)
    )


def get_current_joint_timeout(node) -> list[float]:
    """Read current joints without the SDK wrapper's unbounded service wait."""
    from dsr_msgs2.srv import GetCurrentPosj

    client = node.create_client(GetCurrentPosj, "aux_control/get_current_posj")
    try:
        if not client.wait_for_service(timeout_sec=CURRENT_JOINT_TIMEOUT_SEC):
            raise RuntimeError("Current joint service unavailable")
        future = client.call_async(GetCurrentPosj.Request())
        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=CURRENT_JOINT_TIMEOUT_SEC,
        )
        if not future.done():
            raise RuntimeError("Current joint service response timeout")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("Current joint service rejected request")
        return normalize_pose(response.pos)
    finally:
        node.destroy_client(client)


def set_outputs_compat(node, set_digital_outputs_fn=None, set_digital_output_fn=None):
    """Provide one timeout-safe multi-channel gripper output callable."""
    if set_digital_outputs_fn is not None:
        return set_digital_outputs_fn

    if set_digital_output_fn is None:
        raise RuntimeError("No gripper digital output API available")

    # 구형 SDK의 set_digital_output()은 서비스 응답을 제한시간 없이 기다린다.
    # 응답이 유실되면 작업 노드 전체가 영구 정지하므로 같은 서비스를 직접 호출해
    # 채널별 제한시간과 결과 검사를 적용한다.
    from dsr_msgs2.srv import SetCtrlBoxDigitalOutput

    del set_digital_output_fn
    client = node.create_client(
        SetCtrlBoxDigitalOutput,
        "io/set_ctrl_box_digital_output",
    )

    def _set_outputs(channels: list[int]) -> None:
        """Apply signed channel values through bounded ROS service calls."""
        if not client.wait_for_service(timeout_sec=DIGITAL_OUTPUT_TIMEOUT_SEC):
            raise RuntimeError("Digital output service unavailable")

        for channel in channels:
            index = abs(int(channel))
            value = 1 if int(channel) > 0 else 0
            last_error = "unknown error"

            # 출력 설정은 같은 값을 반복해도 결과가 동일하므로, 일시적인 ROS 서비스
            # 응답 지연에는 제한된 횟수만 재시도하고 영구 대기는 방지한다.
            for attempt in range(1, DIGITAL_OUTPUT_MAX_ATTEMPTS + 1):
                request = SetCtrlBoxDigitalOutput.Request()
                request.index = index
                request.value = value
                future = client.call_async(request)
                rclpy.spin_until_future_complete(
                    node,
                    future,
                    timeout_sec=DIGITAL_OUTPUT_TIMEOUT_SEC,
                )

                if not future.done():
                    future.cancel()
                    last_error = "response timeout"
                else:
                    try:
                        response = future.result()
                    except Exception as exc:
                        last_error = f"service exception: {exc}"
                    else:
                        if response is not None and response.success:
                            if attempt > 1:
                                node.get_logger().info(
                                    "Digital output retry succeeded: "
                                    f"channel={index}, value={value}, attempt={attempt}"
                                )
                            break
                        last_error = "controller rejected request"

                if attempt < DIGITAL_OUTPUT_MAX_ATTEMPTS:
                    node.get_logger().warn(
                        "Digital output request failed; retrying: "
                        f"channel={index}, value={value}, attempt={attempt}/"
                        f"{DIGITAL_OUTPUT_MAX_ATTEMPTS}, error={last_error}"
                    )
                    time.sleep(DIGITAL_OUTPUT_RETRY_DELAY_SEC)
            else:
                raise RuntimeError(
                    "Digital output failed after retries: "
                    f"channel={index}, value={value}, "
                    f"attempts={DIGITAL_OUTPUT_MAX_ATTEMPTS}, error={last_error}"
                )

    node.get_logger().info("Using timeout-safe set_digital_output compatibility wrapper")
    return _set_outputs


def gripper_command(node, set_digital_outputs, wait, name: str, channels: list[int]) -> None:
    """Apply a named gripper I/O pattern and allow the fingers to settle."""
    node.get_logger().info(f"Gripper command: {name} -> {channels}")
    set_digital_outputs(channels)
    wait(DEFAULT_WAIT)


def prepare_home_joint(node, set_digital_outputs, wait) -> None:
    """Partially open the gripper to reduce collision risk during HOME travel."""
    # 홈 복귀 전 그리퍼 간섭을 줄이기 위해 넓게 열린 상태로 맞춘다.
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "HOME_PREPARE_OPEN_0100",
        GRIPPER_HOME_PREPARE,
    )


def move_j(
    node,
    movej,
    posj,
    wait,
    name: str,
    joint_values: list[float],
    vel: float = DEFAULT_VELOCITY,
    acc: float = DEFAULT_ACCELERATION,
    check_motion=None,
) -> None:
    """Execute a checked joint-space move for taught posture transitions."""
    # 조인트 기반 이동. 큰 자세 전환이나 IK 해를 고정하고 싶을 때 사용.
    # 작업 시작점의 HOME 명령만 현재 자세가 이미 HOME이면 생략한다.
    # HOME_RETURN과 일반 경유점은 작업 완료/경로 보장을 위해 항상 실행한다.
    if name == "HOME" and joint_values == HOME_JOINT:
        try:
            current_joint = get_current_joint_timeout(node)
            if is_home_joint(current_joint):
                node.get_logger().info(
                    f"MoveJ skip: HOME already reached -> {current_joint}"
                )
                return
        except Exception as exc:
            node.get_logger().warn(
                f"HOME state check failed; execute HOME move: {exc}"
            )

    target = posj(pose(joint_values))
    node.get_logger().info(f"MoveJ start: {name} -> {joint_values} @ vel={vel}, acc={acc}")
    ensure_motion_allowed()
    ret = movej(target, vel=vel, acc=acc)
    node.get_logger().info(f"MoveJ ret: {name} = {ret}")
    if ret not in (None, 0):
        raise RuntimeError(f"MoveJ failed: {name}, ret={ret}, joint={joint_values}")
    if check_motion is not None:
        while check_motion() == 2:
            wait(0.05)
    wait(DEFAULT_WAIT)


def move_l(
    node,
    movel,
    posx,
    wait,
    name: str,
    pose_values: list[float],
    vel: float = DEFAULT_VELOCITY,
    acc: float = DEFAULT_ACCELERATION,
    check_motion=None,
) -> None:
    """Execute a straight TCP move where the end-effector path matters."""
    # TCP 직선 이동. 집기/삽입처럼 경로 자체가 중요한 구간에 사용.
    target_pose = pose(pose_values)
    node.get_logger().info(f"MoveL start: {name} -> {target_pose} @ vel={vel}, acc={acc}")
    ensure_motion_allowed()
    movel(posx(target_pose), vel=vel, acc=acc)
    if check_motion is not None:
        while check_motion() == 2:
            wait(0.05)
    wait(DEFAULT_WAIT)


def move_up_from_current(
    node,
    get_current_posx,
    movel,
    posx,
    wait,
    dr_base,
    z_offset: float = DEFAULT_HOME_Z_OFFSET,
    vel: float = DEFAULT_VELOCITY,
    acc: float = DEFAULT_ACCELERATION,
) -> None:
    """Lift vertically from the current TCP pose before a large posture change."""
    # 현재 TCP 자세에서 base Z 방향으로 먼저 들어 올린 뒤 홈으로 가기 위한 준비 동작.
    node.get_logger().info(f"Move up {z_offset:.1f} mm from current TCP pose")
    current_pose = normalize_pose(get_current_posx(ref=dr_base))
    target_pose = current_pose.copy()
    target_pose[2] += float(z_offset)

    node.get_logger().info(
        f"Current pose z={current_pose[2]:.3f} -> target z={target_pose[2]:.3f}"
    )
    ensure_motion_allowed()
    movel(posx(target_pose), vel=vel, acc=acc)
    wait(DEFAULT_WAIT)


def go_home(
    node,
    set_digital_outputs,
    wait,
    get_current_posx,
    movel,
    movej,
    posx,
    posj,
    dr_base,
) -> None:
    """Run the common safe HOME flow: open, lift vertically, then joint HOME."""
    # 기본 홈 복귀 절차: 그리퍼 열기 -> 현재 위치에서 위로 상승 -> 홈 조인트 이동.
    node.get_logger().info("Step 1/3: open gripper")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "HOME_RETURN_OPEN_0100",
        GRIPPER_HOME_PREPARE,
    )

    node.get_logger().info("Step 2/3: move up before home")
    move_up_from_current(
        node=node,
        get_current_posx=get_current_posx,
        movel=movel,
        posx=posx,
        wait=wait,
        dr_base=dr_base,
        vel=HOME_MOVE_VELOCITY,
        acc=HOME_MOVE_ACCELERATION,
    )

    node.get_logger().info(f"Step 3/3: move to home joint {HOME_JOINT}")
    prepare_home_joint(node, set_digital_outputs, wait)
    ensure_motion_allowed()
    movej(
        posj(HOME_JOINT),
        vel=HOME_MOVE_VELOCITY,
        acc=HOME_MOVE_ACCELERATION,
    )
    wait(DEFAULT_WAIT)
