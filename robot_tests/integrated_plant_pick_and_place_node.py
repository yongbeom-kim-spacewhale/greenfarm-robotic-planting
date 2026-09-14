# 통합본 전용 식물 심기 노드다. 대상 tray와 source plant 번호를 인자로 받아 1회 실행한다.
from __future__ import annotations

import argparse
import sys

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import set_outputs_compat
from robot_tests.node_common import (
    ROBOT_ID,
    configure_dsr_init,
    import_basic_motion_apis,
    import_digital_output_apis,
)
from robot_tests.plant_pick_and_place_common import (
    TARGET_TRAY_PLACE_JOINTS,
    TCP_NAME,
    TOOL_NAME,
    run_plant_pick_and_place_sequence,
)
from robot_tests.robot_motion_data import PLANT_ACTIVE_TRAY, TRAY_PLANT_JOINTS

bootstrap_dsr_python()

import DR_init

configure_dsr_init(DR_init)

TRAY_LABEL_TO_KEY = {
    "A": "tray_a",
    "B": "tray_b",
    "C": "tray_c",
    "D": "tray_d",
}


def parse_cli_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Validate the dashboard-selected source plant and destination tray."""
    valid_plants = sorted(TRAY_PLANT_JOINTS[PLANT_ACTIVE_TRAY].keys())

    parser = argparse.ArgumentParser(description="Integrated plant pick and place node")
    parser.add_argument(
        "--tray",
        required=True,
        choices=sorted(TRAY_LABEL_TO_KEY.keys()),
        help="Target tray label",
    )
    parser.add_argument(
        "--plant",
        required=True,
        type=int,
        choices=valid_plants,
        help="Source plant index from active tray",
    )
    cli_args = rclpy.utilities.remove_ros_args(args=argv or sys.argv)[1:]
    return parser.parse_args(cli_args)


def main(args=None):
    """Configure the plant gripper and execute one dashboard planting request."""
    parsed_args = parse_cli_args(args)
    rclpy.init(args=args)

    node = rclpy.create_node("integrated_plant_pick_and_place", namespace=ROBOT_ID)
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

        target_tray = TRAY_LABEL_TO_KEY[parsed_args.tray]
        plant_index = parsed_args.plant

        node.get_logger().info(
            f"Selected source={PLANT_ACTIVE_TRAY}, plant={plant_index}, target={parsed_args.tray}"
        )
        run_plant_pick_and_place_sequence(
            node=node,
            movej=motion_apis["movej"],
            movel=motion_apis["movel"],
            posj=motion_apis["posj"],
            posx=motion_apis["posx"],
            set_digital_outputs=set_digital_outputs,
            wait=motion_apis["wait"],
            get_current_posx=motion_apis["get_current_posx"],
            dr_base=motion_apis["DR_BASE"],
            source_tray=PLANT_ACTIVE_TRAY,
            plant_index=plant_index,
            target_tray=target_tray,
        )
        node.get_logger().info(
            "integrated_plant_pick_and_place complete; "
            f"final place joint={TARGET_TRAY_PLACE_JOINTS[target_tray]['down']}"
        )

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"integrated_plant_pick_and_place failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
