"""Persistent and one-shot manual control node for the integrated dashboard."""

from __future__ import annotations

import argparse
import json
import sys

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import move_j, move_l
from robot_tests.node_common import ROBOT_ID, configure_dsr_init, setup_tool_and_tcp
from robot_tests.robot_motion_data import PLANT_TCP_NAME, PLANT_TOOL_NAME

bootstrap_dsr_python()

import DR_init

configure_dsr_init(DR_init)

DEFAULT_VEL = 30.0
DEFAULT_ACC = 30.0
COMMAND_MODES = (
    "read-joint",
    "move-joint",
    "read-base",
    "move-base",
    "recover-config",
    "server",
)


def call_read_service(node, service_type, service_name: str, request, timeout_sec: float = 3.0):
    """Call a controller read service with a finite dashboard-friendly timeout."""
    client = node.create_client(service_type, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service unavailable: {service_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(
                f"Service response timeout after {timeout_sec:.1f}s: {service_name}"
            )
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"Service rejected: {service_name}")
        return response
    finally:
        node.destroy_client(client)


def parse_cli_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Select persistent stdin-server mode or a single manual command."""
    parser = argparse.ArgumentParser(description="Integrated manual control node")
    parser.add_argument("--mode", required=True, choices=COMMAND_MODES)
    parser.add_argument("--joints", nargs=6, type=float)
    parser.add_argument("--pose", nargs=6, type=float)
    cli_args = rclpy.utilities.remove_ros_args(args=argv or sys.argv)[1:]
    return parser.parse_args(cli_args)


def execute_command(node, apis: dict, command: dict) -> None:
    """Execute one read/move/recovery command received from the dashboard."""
    mode = str(command["mode"])
    if mode == "read-joint":
        from dsr_msgs2.srv import GetCurrentPosj

        response = call_read_service(
            node,
            GetCurrentPosj,
            "aux_control/get_current_posj",
            GetCurrentPosj.Request(),
        )
        joint = [float(value) for value in response.pos]
        node.get_logger().info(
            "joint_position_json="
            + json.dumps(joint, ensure_ascii=False, separators=(",", ":"))
        )
    elif mode == "read-base":
        from dsr_msgs2.srv import GetCurrentPosx

        request = GetCurrentPosx.Request()
        request.ref = int(apis["DR_BASE"])
        response = call_read_service(
            node,
            GetCurrentPosx,
            "aux_control/get_current_posx",
            request,
        )
        if not response.task_pos_info:
            raise RuntimeError("Current base pose response is empty")
        pose = [float(value) for value in response.task_pos_info[0].data[:6]]
        node.get_logger().info(
            "base_pose_json="
            + json.dumps(pose, ensure_ascii=False, separators=(",", ":"))
        )
    elif mode == "move-joint":
        joints = command.get("joints")
        if not isinstance(joints, list) or len(joints) != 6:
            raise ValueError("Six joint values are required")
        move_j(
            node,
            apis["movej"],
            apis["posj"],
            apis["wait"],
            "MANUAL_MOVE_JOINT",
            [float(value) for value in joints],
            vel=DEFAULT_VEL,
            acc=DEFAULT_ACC,
        )
    elif mode == "move-base":
        pose = command.get("pose")
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError("Six base pose values are required")
        move_l(
            node,
            apis["movel"],
            apis["posx"],
            apis["wait"],
            "MANUAL_MOVE_BASE",
            [float(value) for value in pose],
            vel=DEFAULT_VEL,
            acc=DEFAULT_ACC,
        )
    elif mode == "recover-config":
        setup_tool_and_tcp(node, None, None, PLANT_TOOL_NAME, PLANT_TCP_NAME)
        node.get_logger().info(
            f"recovery_config complete tool={PLANT_TOOL_NAME}, tcp={PLANT_TCP_NAME}"
        )
    else:
        raise ValueError(f"Unsupported manual command mode: {mode}")


def emit_result(node, mode: str, success: bool, error: str = "") -> None:
    """Print a stable JSON marker consumed by the dashboard output parser."""
    payload = {"mode": mode, "success": success, "error": error}
    node.get_logger().info(
        "manual_command_result_json="
        + json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    )


def run_server(node, apis: dict) -> None:
    """Process newline-delimited JSON commands without restarting the ROS node."""
    setup_tool_and_tcp(node, None, None, PLANT_TOOL_NAME, PLANT_TCP_NAME)
    node.get_logger().info(
        f"startup_config complete tool={PLANT_TOOL_NAME}, tcp={PLANT_TCP_NAME}"
    )
    node.get_logger().info("manual_control_server_ready")
    for raw_line in sys.stdin:
        if not raw_line.strip():
            continue
        mode = "unknown"
        try:
            command = json.loads(raw_line)
            mode = str(command.get("mode", "unknown"))
            execute_command(node, apis, command)
            emit_result(node, mode, True)
        except Exception as exc:
            node.get_logger().error(f"manual command failed mode={mode}: {exc}")
            emit_result(node, mode, False, str(exc))


def main(args=None):
    """Create the manual-control ROS context and run server or one-shot mode."""
    parsed = parse_cli_args(args)
    rclpy.init(args=args)
    node = rclpy.create_node("integrated_manual_control", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import DR_BASE, movej, movel, wait
        from DR_common2 import posj, posx

        apis = {
            "DR_BASE": DR_BASE,
            "movej": movej,
            "movel": movel,
            "wait": wait,
            "posj": posj,
            "posx": posx,
        }
        if parsed.mode == "server":
            run_server(node, apis)
        else:
            command = {"mode": parsed.mode}
            if parsed.joints is not None:
                command["joints"] = list(parsed.joints)
            if parsed.pose is not None:
                command["pose"] = list(parsed.pose)
            execute_command(node, apis, command)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"integrated_manual_control failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
