"""Doosan M0609 shovel pickup sequence node."""

from __future__ import annotations

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import (
    GRIPPER_PICK_CLOSE,
    HOME_JOINT,
    move_j,
    move_up_from_current,
    prepare_home_joint,
    set_outputs_compat,
    gripper_command,
)
from robot_tests.node_common import (
    ROBOT_ID,
    configure_dsr_init,
    import_basic_motion_apis,
    import_digital_output_apis,
)
from robot_tests.robot_motion_data import (
    SHOVEL_PICK_JOINT,
    SHOVEL_READY_JOINT,
    SHOVEL_TCP_NAME,
    SHOVEL_TOOL_NAME,
)

bootstrap_dsr_python()

import DR_init

configure_dsr_init(DR_init)

FAST_VEL = 80.0
FAST_ACC = 80.0
SLOW_VEL = 20.0
SLOW_ACC = 20.0


def run_sequence(node, movej, movel, posj, posx, get_current_posx, dr_base, set_digital_outputs, wait) -> None:
    node.get_logger().info("Step 1/5: go home safely")
    prepare_home_joint(node, set_digital_outputs, wait)
    move_j(node, movej, posj, wait, "HOME", HOME_JOINT, vel=FAST_VEL, acc=FAST_ACC)

    node.get_logger().info("Step 2/5: move to shovel ready joint")
    move_j(
        node,
        movej,
        posj,
        wait,
        "SHOVEL_READY",
        SHOVEL_READY_JOINT,
        vel=FAST_VEL,
        acc=FAST_ACC,
    )

    node.get_logger().info("Step 3/5: wide open gripper")
    wait(1.0)
    prepare_home_joint(node, set_digital_outputs, wait)

    node.get_logger().info("Step 4/5: move to shovel pick joint and grip")
    move_j(
        node,
        movej,
        posj,
        wait,
        "SHOVEL_PICK",
        SHOVEL_PICK_JOINT,
        vel=SLOW_VEL,
        acc=SLOW_ACC,
    )
    wait(1.0)
    # 닫기 명령 직전부터는 삽 장착 여부가 불확실할 수 있는 구간이다.
    node.get_logger().info("tool_phase=SHOVEL_PICKING_GRIP")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "SHOVEL_PICK_CLOSE_0010",
        GRIPPER_PICK_CLOSE,
    )
    node.get_logger().info("tool_phase=SHOVEL_HELD")

    node.get_logger().info("Step 5/5: move up 120 mm, then return home with shovel gripped")
    move_up_from_current(
        node=node,
        get_current_posx=get_current_posx,
        movel=movel,
        posx=posx,
        wait=wait,
        dr_base=dr_base,
        z_offset=120.0,
    )
    move_j(
        node,
        movej,
        posj,
        wait,
        "HOME_RETURN",
        HOME_JOINT,
        vel=FAST_VEL,
        acc=FAST_ACC,
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("shovel_pickup", namespace=ROBOT_ID)
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
            get_current_posx=motion_apis["get_current_posx"],
            dr_base=motion_apis["DR_BASE"],
            set_digital_outputs=set_digital_outputs,
            wait=motion_apis["wait"],
        )
        node.get_logger().info("shovel_pickup complete")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"shovel_pickup failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
