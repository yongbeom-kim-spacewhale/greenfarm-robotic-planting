"""Common ROS/Doosan node helpers for robot test nodes."""

from __future__ import annotations

from typing import Any

import rclpy

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


def configure_dsr_init(dr_init_module: Any) -> None:
    """Set robot id/model globals before the Doosan SDK binds its ROS services."""
    dr_init_module.__dsr__id = ROBOT_ID
    dr_init_module.__dsr__model = ROBOT_MODEL


def import_basic_motion_apis(node):
    """Load the joint, Cartesian, pose and Tool/TCP APIs used by motion nodes."""
    try:
        from DSR_ROBOT2 import (
            DR_BASE,
            get_current_posj,
            get_current_posx,
            movej,
            movel,
            set_tcp,
            set_tool,
            wait,
        )
        from DR_common2 import posj, posx
    except ImportError as exc:
        node.get_logger().error(f"Failed to import Doosan API: {exc}")
        node.destroy_node()
        raise

    try:
        from DSR_ROBOT2 import check_motion
    except ImportError:
        check_motion = None

    return {
        "DR_BASE": DR_BASE,
        "check_motion": check_motion,
        "get_current_posj": get_current_posj,
        "get_current_posx": get_current_posx,
        "movej": movej,
        "movel": movel,
        "set_tcp": set_tcp,
        "set_tool": set_tool,
        "wait": wait,
        "posj": posj,
        "posx": posx,
    }


def import_digital_output_apis():
    """Return current and legacy controller I/O functions for gripper control."""
    try:
        from DSR_ROBOT2 import set_digital_outputs as set_digital_outputs_fn
    except ImportError:
        set_digital_outputs_fn = None

    try:
        from DSR_ROBOT2 import set_digital_output as set_digital_output_fn
    except ImportError:
        set_digital_output_fn = None

    return set_digital_outputs_fn, set_digital_output_fn


def _set_named_robot_config(
    node,
    service_type,
    service_name: str,
    value: str,
    *,
    timeout_sec: float = 5.0,
) -> bool:
    """Set one controller configuration without waiting forever on the SDK wrapper."""
    client = node.create_client(service_type, service_name)
    try:
        node.get_logger().info(f"Waiting for {service_name} service")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn(f"Service unavailable: {service_name}; continuing")
            return False

        request = service_type.Request()
        request.name = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

        if not future.done():
            node.get_logger().warn(
                f"Service response timeout after {timeout_sec:.1f}s: "
                f"{service_name}, value={value}; continuing"
            )
            return False
        try:
            response = future.result()
        except Exception as exc:
            node.get_logger().warn(
                f"Service call failed: {service_name}, error={exc}; continuing"
            )
            return False
        if response is None or not response.success:
            node.get_logger().warn(
                f"Service rejected: {service_name}, value={value}; "
                "keep controller's current setting"
            )
            return False
        return True
    finally:
        node.destroy_client(client)


def setup_tool_and_tcp(node, set_tool, set_tcp, tool_name: str, tcp_name: str) -> None:
    """Synchronise controller Tool/TCP names before executing taught poses."""
    # set_tool/set_tcp 인자는 기존 호출부 호환을 위해 유지한다. SDK 함수는 응답 제한시간이
    # 없어 controller가 응답하지 않으면 노드가 영구 대기하므로 timeout 가능한 직접 서비스를 쓴다.
    del set_tool, set_tcp
    from dsr_msgs2.srv import SetCurrentTcp, SetCurrentTool

    tool_configured = _set_named_robot_config(
        node,
        SetCurrentTool,
        "tool/set_current_tool",
        tool_name,
    )
    if tool_configured:
        node.get_logger().info(f"Configured tool={tool_name}")
    tcp_configured = _set_named_robot_config(
        node,
        SetCurrentTcp,
        "tcp/set_current_tcp",
        tcp_name,
    )
    if tcp_configured:
        node.get_logger().info(f"Configured tcp={tcp_name}")
