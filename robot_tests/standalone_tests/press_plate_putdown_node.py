# 집은 프레스 판을 목표 위치에 내려놓는 순서를 테스트하는 코드다.
"""Doosan M0609 press plate put-down sequence node."""

from __future__ import annotations

import argparse
import sys

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import (
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
    PRESS_PLATE_PLACE_POSE,
    PRESS_PLATE_TOP_POSE,
    PRESS_TCP_NAME,
    PRESS_TOOL_NAME,
)

bootstrap_dsr_python()

import DR_init

# TODO: Replace these route/task poses with measured production coordinates.
MID_POSE = PRESS_MID_POSE
PLATE_TOP_POSE = PRESS_PLATE_TOP_POSE
PLATE_PLACE_POSE = PRESS_PLATE_PLACE_POSE

TOOL_NAME = PRESS_TOOL_NAME
TCP_NAME = PRESS_TCP_NAME

configure_dsr_init(DR_init)


def parse_cli_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Press plate put-down sequence")
    parser.add_argument(
        "--skip-initial-home",
        action="store_true",
        help="Skip HOME move when the previous dashboard task already ended at HOME",
    )
    cli_args = rclpy.utilities.remove_ros_args(args=argv or sys.argv)[1:]
    return parser.parse_args(cli_args)


def run_sequence(
    node,
    movej,
    movel,
    posj,
    posx,
    set_digital_outputs,
    wait,
    check_motion,
    *,
    skip_initial_home: bool = False,
) -> None:
    if skip_initial_home:
        node.get_logger().info("Step 1/8: skip HOME; previous task already ended at HOME")
    else:
        node.get_logger().info("Step 1/8: go home safely with plate gripped")
        move_j(node, movej, posj, wait, "HOME", HOME_JOINT, check_motion=check_motion)

    node.get_logger().info("Step 2/8: move to home route point")
    move_l(node, movel, posx, wait, "MID_ROUTE_FROM_HOME", MID_POSE, check_motion=check_motion)

    node.get_logger().info("Step 3/8: move above press plate place position")
    move_l(node, movel, posx, wait, "PLATE_TOP", PLATE_TOP_POSE, check_motion=check_motion)

    node.get_logger().info("Step 4/8: move down to place pose")
    move_l(node, movel, posx, wait, "PLATE_PLACE", PLATE_PLACE_POSE, check_motion=check_motion)

    node.get_logger().info("Step 5/8: release press plate")
    # 해제 명령 도중 정지되면 누름판이 빠졌는지 확정할 수 없도록 중간 상태를 알린다.
    node.get_logger().info("tool_phase=PRESS_PUTTING_RELEASE")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "RELEASE_MID_OPEN_1000",
        PLANT_GRIPPER_WAYPOINT1_PREPARE,
    )
    # 열기 명령이 정상 반환된 뒤에만 툴 없음 상태로 확정한다.
    node.get_logger().info("tool_phase=PRESS_RELEASED")

    node.get_logger().info("Step 6/8: move back to top pose")
    move_l(node, movel, posx, wait, "PLATE_TOP_RETURN", PLATE_TOP_POSE, check_motion=check_motion)

    node.get_logger().info("Step 7/8: move back through middle route")
    move_l(node, movel, posx, wait, "MID_ROUTE_TO_HOME", MID_POSE, check_motion=check_motion)

    node.get_logger().info("Step 8/8: return home")
    prepare_home_joint(node, set_digital_outputs, wait)
    move_j(node, movej, posj, wait, "HOME_RETURN", HOME_JOINT, check_motion=check_motion)


def main(args: list[str] | None = None) -> None:
    parsed_args = parse_cli_args(args)
    rclpy.init(args=args)
    node = rclpy.create_node("press_plate_putdown", namespace=ROBOT_ID)
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
            skip_initial_home=parsed_args.skip_initial_home,
        )
        node.get_logger().info("press_plate_putdown complete")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"press_plate_putdown failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
