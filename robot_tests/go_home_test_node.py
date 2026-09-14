# 로봇을 현재 위치에서 안전하게 홈 자세로 복귀시키는 테스트 코드다.
"""Doosan M0609 safe go-home test node."""

from __future__ import annotations

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import HOME_JOINT, go_home, set_outputs_compat
from robot_tests.node_common import (
    ROBOT_ID,
    ROBOT_MODEL,
    configure_dsr_init,
    import_basic_motion_apis,
    import_digital_output_apis,
)
from robot_tests.robot_motion_data import PRESS_TCP_NAME, PRESS_TOOL_NAME

bootstrap_dsr_python()

import DR_init

# 홈 복귀 테스트에서 사용할 툴/툴센터포인트 설정.
TOOL_NAME = PRESS_TOOL_NAME
TCP_NAME = PRESS_TCP_NAME

configure_dsr_init(DR_init)


def main(args: list[str] | None = None) -> None:
    """Run the dashboard HOME sequence: configure robot, open, lift, and move HOME."""
    rclpy.init(args=args)
    node = rclpy.create_node("m0609_go_home_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        # 현재 TCP 위치를 읽고 안전 홈 복귀를 수행하는 데 필요한 API들.
        motion_apis = import_basic_motion_apis(node)
    except ImportError:
        node.destroy_node()
        rclpy.shutdown()
        return

    set_digital_outputs_fn, set_digital_output_fn = import_digital_output_apis()

    try:
        # 홈 복귀 전 tool / tcp를 맞춘 뒤 공용 go_home 절차를 실행한다.
        set_digital_outputs = set_outputs_compat(
            node,
            set_digital_outputs_fn=set_digital_outputs_fn,
            set_digital_output_fn=set_digital_output_fn,
        )
        go_home(
            node=node,
            set_digital_outputs=set_digital_outputs,
            wait=motion_apis["wait"],
            get_current_posx=motion_apis["get_current_posx"],
            movel=motion_apis["movel"],
            movej=motion_apis["movej"],
            posx=motion_apis["posx"],
            posj=motion_apis["posj"],
            dr_base=motion_apis["DR_BASE"],
        )
        node.get_logger().info("go_home complete")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"go_home failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
