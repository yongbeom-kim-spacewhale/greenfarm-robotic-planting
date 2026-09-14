# 통합본 전용 트레이 평탄화 노드다. tray 인자를 받아 평탄화 1회를 실행한다.
from __future__ import annotations

import argparse
import json
import sys

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.node_common import ROBOT_ID, configure_dsr_init, setup_tool_and_tcp
from robot_tests.robot_motion_data import (
    COMMON_TCP_NAME,
    COMMON_TOOL_NAME,
    TRAY_SOIL_TCP_NAME,
    TRAY_SOIL_TOOL_NAME,
)
from robot_tests.tray_soil_service_common import (
    go_home_with_gripper_held,
    import_soil_service_apis,
    run_tray_flatten_action,
)

bootstrap_dsr_python()

import DR_init

configure_dsr_init(DR_init)


def parse_cli_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Read the tray selected by the dashboard flatten action."""
    parser = argparse.ArgumentParser(description="Integrated tray soil flatten node")
    parser.add_argument(
        "--tray",
        required=True,
        choices=["A", "B", "C", "D"],
        help="Tray label to flatten",
    )
    cli_args = rclpy.utilities.remove_ros_args(args=argv or sys.argv)[1:]
    return parser.parse_args(cli_args)


def main(args=None):
    """Run one compliance flatten cycle and restore common Tool/TCP settings."""
    parsed_args = parse_cli_args(args)
    rclpy.init(args=args)
    node = rclpy.create_node("integrated_tray_soil_flatten", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        apis = import_soil_service_apis(node)
    except ImportError:
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        setup_tool_and_tcp(
            node,
            apis["set_tool"],
            apis["set_tcp"],
            TRAY_SOIL_TOOL_NAME,
            TRAY_SOIL_TCP_NAME,
        )
        go_home_with_gripper_held(node, apis)

        result = run_tray_flatten_action(node, apis, parsed_args.tray)
        node.get_logger().info(
            "measurement_result_json="
            + json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        )

        # 평탄화 함수가 마지막 코너에서 CENTER_TOP_RETURN까지 복귀했으므로
        # 종료 HOME 직전의 추가 120 mm 상승은 생략한다.
        go_home_with_gripper_held(node, apis, move_up=False)
        node.get_logger().info("integrated_tray_soil_flatten finished")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"integrated_tray_soil_flatten failed: {exc}")
        raise
    finally:
        try:
            setup_tool_and_tcp(
                node,
                apis["set_tool"],
                apis["set_tcp"],
                COMMON_TOOL_NAME,
                COMMON_TCP_NAME,
            )
            node.get_logger().info("Restored common Tool/TCP after flatten compliance")
        except Exception as exc:
            node.get_logger().warn(f"Failed to restore common Tool/TCP: {exc}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
