# 프레스 판을 집어 들어 올리는 순서를 테스트하는 코드다.
"""Doosan M0609 press plate pickup sequence node."""

from __future__ import annotations

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import (
    GRIPPER_PICK_CLOSE,
    HOME_JOINT,
    move_j,
    move_l,
    prepare_home_joint,
    set_outputs_compat,
    gripper_command,
)
from robot_tests.node_common import (
    ROBOT_ID,
    ROBOT_MODEL,
    configure_dsr_init,
    import_basic_motion_apis,
    import_digital_output_apis,
)
from robot_tests.robot_motion_data import (
    PLANT_GRIPPER_WAYPOINT1_PREPARE,
    PRESS_MID_POSE,
    PRESS_PLATE_PICK_POSE,
    PRESS_PLATE_TOP_POSE,
    PRESS_TCP_NAME,
    PRESS_TOOL_NAME,
)

bootstrap_dsr_python()

import DR_init

# TODO: 아래 경로/작업 좌표는 실제 측정 생산 좌표로 교체 필요.
# 홈에서 프레스 플레이트 쪽으로 빠져나가는 중간 경유점.
MID_POSE = PRESS_MID_POSE
# 플레이트 상부 대기 위치.
PLATE_TOP_POSE = PRESS_PLATE_TOP_POSE
# 실제 플레이트를 집는 하강 위치.
PLATE_PICK_POSE = PRESS_PLATE_PICK_POSE

# 프레스 플레이트 픽업 시 사용할 툴/툴센터포인트 설정.
TOOL_NAME = PRESS_TOOL_NAME
TCP_NAME = PRESS_TCP_NAME

configure_dsr_init(DR_init)

FAST_VEL = 85.0
FAST_ACC = 80.0
TRANSFER_VEL = 80.0
TRANSFER_ACC = 75.0
PICK_VEL = 40.0
PICK_ACC = 35.0
LIFT_VEL = 50.0
LIFT_ACC = 45.0
RETURN_VEL = 85.0
RETURN_ACC = 80.0


def run_sequence(node, movej, movel, posj, posx, set_digital_outputs, wait, check_motion) -> None:
    # 홈 -> 중간 경유 -> 플레이트 상부 -> 집기 -> 원복귀 순서.
    node.get_logger().info("Step 1/9: go home safely")
    prepare_home_joint(node, set_digital_outputs, wait)
    move_j(
        node,
        movej,
        posj,
        wait,
        "HOME",
        HOME_JOINT,
        vel=FAST_VEL,
        acc=FAST_ACC,
        check_motion=check_motion,
    )

    node.get_logger().info("Step 2/9: move to home route point")
    move_l(
        node,
        movel,
        posx,
        wait,
        "MID_ROUTE_FROM_HOME",
        MID_POSE,
        vel=FAST_VEL,
        acc=FAST_ACC,
        check_motion=check_motion,
    )

    node.get_logger().info("Step 3/9: move above press plate")
    move_l(
        node,
        movel,
        posx,
        wait,
        "PLATE_TOP",
        PLATE_TOP_POSE,
        vel=TRANSFER_VEL,
        acc=TRANSFER_ACC,
        check_motion=check_motion,
    )

    node.get_logger().info("Step 4/9: slightly open gripper")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "MID_OPEN_1000",
        PLANT_GRIPPER_WAYPOINT1_PREPARE,
    )

    node.get_logger().info("Step 5/9: move down to pickup pose")
    move_l(
        node,
        movel,
        posx,
        wait,
        "PLATE_PICK",
        PLATE_PICK_POSE,
        vel=PICK_VEL,
        acc=PICK_ACC,
        check_motion=check_motion,
    )

    node.get_logger().info("Step 6/9: strong close gripper")
    # 이 표식 이후 정지되면 그리퍼 명령의 물리적 완료 여부를 확정할 수 없다.
    node.get_logger().info("tool_phase=PRESS_PICKING_GRIP")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "PICK_CLOSE_0010",
        GRIPPER_PICK_CLOSE,
    )
    # 닫기 명령이 정상 반환된 뒤에만 누름판 보유 상태로 확정한다.
    node.get_logger().info("tool_phase=PRESS_PLATE_HELD")

    node.get_logger().info("Step 7/9: move back to top pose")
    move_l(
        node,
        movel,
        posx,
        wait,
        "PLATE_TOP_RETURN",
        PLATE_TOP_POSE,
        vel=LIFT_VEL,
        acc=LIFT_ACC,
        check_motion=check_motion,
    )

    node.get_logger().info("Step 8/9: move back through middle route")
    move_l(
        node,
        movel,
        posx,
        wait,
        "MID_ROUTE_TO_HOME",
        MID_POSE,
        vel=RETURN_VEL,
        acc=RETURN_ACC,
        check_motion=check_motion,
    )

    node.get_logger().info("Step 9/9: return home while keeping plate gripped")
    move_j(
        node,
        movej,
        posj,
        wait,
        "HOME_RETURN",
        HOME_JOINT,
        vel=RETURN_VEL,
        acc=RETURN_ACC,
        check_motion=check_motion,
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("press_plate_pickup", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        motion_apis = import_basic_motion_apis(node)
    except ImportError:
        node.destroy_node()
        rclpy.shutdown()
        return

    set_digital_outputs_fn, set_digital_output_fn = import_digital_output_apis()

    try:
        set_digital_outputs = set_outputs_compat(
            node,
            set_digital_outputs_fn=set_digital_outputs_fn,
            set_digital_output_fn=set_digital_output_fn,
        )

        run_sequence(
            node=node,
            movej=motion_apis["movej"],
            movel=motion_apis["movel"],
            posj=motion_apis["posj"],
            posx=motion_apis["posx"],
            set_digital_outputs=set_digital_outputs,
            wait=motion_apis["wait"],
            check_motion=motion_apis["check_motion"],
        )
        node.get_logger().info("press_plate_pickup complete")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"press_plate_pickup failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
