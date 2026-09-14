"""Unified dashboard entry point for press-plate and shovel handling."""

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
from robot_tests.standalone_tests.press_plate_pickup_node import run_sequence as run_press_pickup
from robot_tests.standalone_tests.press_plate_putdown_node import run_sequence as run_press_putdown
from robot_tests.standalone_tests.shovel_pickup_node import run_sequence as run_shovel_pickup
from robot_tests.standalone_tests.shovel_putdown_node import run_sequence as run_shovel_putdown

bootstrap_dsr_python()

import DR_init

configure_dsr_init(DR_init)


def parse_cli_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Validate the dashboard's requested tool/action combination."""
    parser = argparse.ArgumentParser(description="Integrated tool pickup/putdown node")
    parser.add_argument("--tool", required=True, choices=("press", "shovel"))
    parser.add_argument("--action", required=True, choices=("pickup", "putdown"))
    parser.add_argument(
        "--skip-initial-home",
        action="store_true",
        help="Skip the initial HOME move for press putdown only",
    )
    cli_args = rclpy.utilities.remove_ros_args(args=argv or sys.argv)[1:]
    args = parser.parse_args(cli_args)
    if args.skip_initial_home and (args.tool, args.action) != ("press", "putdown"):
        parser.error("--skip-initial-home is only valid for press putdown")
    return args


def run_selected_sequence(node, args: argparse.Namespace, motion_apis, set_outputs) -> None:
    """Dispatch one unified dashboard request to the taught tool sequence."""
    common = {
        "node": node,
        "movej": motion_apis["movej"],
        "posj": motion_apis["posj"],
        "set_digital_outputs": set_outputs,
        "wait": motion_apis["wait"],
    }
    selection = (args.tool, args.action)
    if selection == ("press", "pickup"):
        run_press_pickup(
            **common,
            movel=motion_apis["movel"],
            posx=motion_apis["posx"],
            check_motion=motion_apis["check_motion"],
        )
    elif selection == ("press", "putdown"):
        run_press_putdown(
            **common,
            movel=motion_apis["movel"],
            posx=motion_apis["posx"],
            check_motion=motion_apis["check_motion"],
            skip_initial_home=args.skip_initial_home,
        )
    elif selection == ("shovel", "pickup"):
        run_shovel_pickup(
            **common,
            movel=motion_apis["movel"],
            posx=motion_apis["posx"],
            get_current_posx=motion_apis["get_current_posx"],
            dr_base=motion_apis["DR_BASE"],
        )
    else:
        run_shovel_putdown(
            **common,
            check_motion=motion_apis["check_motion"],
        )


def main(args: list[str] | None = None) -> None:
    """Configure one ROS/Doosan context and run exactly one tool operation."""
    parsed_args = parse_cli_args(args)
    rclpy.init(args=args)
    node = rclpy.create_node("integrated_tool_control", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        motion_apis = import_basic_motion_apis(node)
    except ImportError:
        node.destroy_node()
        rclpy.shutdown()
        return

    set_digital_outputs_fn, set_digital_output_fn = import_digital_output_apis()
    try:
        set_outputs = set_outputs_compat(
            node,
            set_digital_outputs_fn=set_digital_outputs_fn,
            set_digital_output_fn=set_digital_output_fn,
        )
        run_selected_sequence(node, parsed_args, motion_apis, set_outputs)
        node.get_logger().info(
            f"integrated_tool_control complete: {parsed_args.tool} {parsed_args.action}"
        )
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"integrated_tool_control failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
