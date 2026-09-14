"""Doosan M0609 shovel putdown sequence node."""

from __future__ import annotations

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import (
    HOME_JOINT,
    move_j,
    prepare_home_joint,
    set_outputs_compat,
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
READY_STABILIZE_WAIT = 1.0


def run_sequence(node, movej, posj, set_digital_outputs, wait, check_motion) -> None:
    node.get_logger().info("Step 1/5: go home with shovel gripped")
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
        check_motion=check_motion,
    )
    node.get_logger().info(f"Stabilize at shovel ready joint for {READY_STABILIZE_WAIT:.1f}s")
    wait(READY_STABILIZE_WAIT)

    node.get_logger().info("Step 3/5: move to shovel putdown joint")
    move_j(
        node,
        movej,
        posj,
        wait,
        "SHOVEL_PUTDOWN",
        SHOVEL_PICK_JOINT,
        vel=SLOW_VEL,
        acc=SLOW_ACC,
        check_motion=check_motion,
    )

    node.get_logger().info("Step 4/5: open gripper and release shovel")
    wait(1.0)
    # 열기 명령 직전부터는 삽 해제 여부가 불확실할 수 있는 구간이다.
    node.get_logger().info("tool_phase=SHOVEL_PUTTING_RELEASE")
    prepare_home_joint(node, set_digital_outputs, wait)
    node.get_logger().info("tool_phase=SHOVEL_RELEASED")

    node.get_logger().info("Step 5/5: return home")
    move_j(
        node,
        movej,
        posj,
        wait,
        "HOME_RETURN",
        HOME_JOINT,
        vel=FAST_VEL,
        acc=FAST_ACC,
        check_motion=check_motion,
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("shovel_putdown", namespace=ROBOT_ID)
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
            posj=motion_apis["posj"],
            set_digital_outputs=set_digital_outputs,
            wait=motion_apis["wait"],
            check_motion=motion_apis["check_motion"],
        )
        node.get_logger().info("shovel_putdown complete")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"shovel_putdown failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
