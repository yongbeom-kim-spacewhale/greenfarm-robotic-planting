# 통합본 전용 다음 작업 노드다. tray, soil 상태, 다음 작업을 받아 1단계 처리한다.
from __future__ import annotations

import argparse
import json
import sys

import rclpy

from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import set_outputs_compat
from robot_tests.node_common import (
    ROBOT_ID,
    configure_dsr_init,
    import_basic_motion_apis,
    import_digital_output_apis,
    setup_tool_and_tcp,
)
from robot_tests.plant_pick_and_place_common import (
    TARGET_TRAY_PLACE_JOINTS,
    TCP_NAME as PLANT_TCP_NAME,
    TOOL_NAME as PLANT_TOOL_NAME,
    run_plant_pick_and_place_sequence,
)
from robot_tests.robot_motion_data import (
    COMMON_TCP_NAME,
    COMMON_TOOL_NAME,
    PLANT_ACTIVE_TRAY,
    SOIL_SERVICE_TCP_NAME,
    SOIL_SERVICE_TOOL_NAME,
    TRAY_SOIL_TCP_NAME,
    TRAY_SOIL_TOOL_NAME,
)
from robot_tests.tray_soil_service_common import (
    go_home_with_gripper_held,
    import_soil_service_apis,
    inspect_tray_soil,
    require_rock_candidate,
    run_rock_remove_action,
    run_soil_add_action,
    run_soil_remove_action,
    run_tray_flatten_action,
)

bootstrap_dsr_python()

import DR_init

configure_dsr_init(DR_init)

SOIL_STATUS_TO_RESULT = {
    "흙부족": "SOIL_LOW",
    "정상": "SOIL_OK",
    "흙많음": "SOIL_HIGH",
    "장애물감지": "SKIPPED_DUE_TO_ROCK",
}

NEXT_ACTION_MEASURE = "토양 측정"
NEXT_ACTION_ADD_SOIL = "흙추가"
NEXT_ACTION_REMOVE_SOIL = "흙제거"
NEXT_ACTION_REMOVE_ROCK = "돌제거"
NEXT_ACTION_FLATTEN = "평탄화"
NEXT_ACTION_PLANT = "식물 심기"

TRAY_LABEL_TO_KEY = {
    "A": "tray_a",
    "B": "tray_b",
    "C": "tray_c",
    "D": "tray_d",
}


def parse_cli_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Read the tray state snapshot supplied by the dashboard process."""
    parser = argparse.ArgumentParser(description="Integrated tray next step node")
    parser.add_argument("--tray", required=True, choices=["A", "B", "C", "D"])
    parser.add_argument(
        "--soil-status",
        required=True,
        choices=["미측정", "흙부족", "정상", "흙많음", "장애물감지", "측정실패"],
    )
    parser.add_argument(
        "--next-action",
        required=True,
        choices=[
            NEXT_ACTION_MEASURE,
            NEXT_ACTION_ADD_SOIL,
            NEXT_ACTION_REMOVE_SOIL,
            NEXT_ACTION_REMOVE_ROCK,
            NEXT_ACTION_FLATTEN,
            NEXT_ACTION_PLANT,
            "완료",
        ],
    )
    parser.add_argument("--plant", type=int, help="Plant index for plant action")
    parser.add_argument("--measurement-json", help="Serialized last measurement result")
    parser.add_argument(
        "--skip-initial-home",
        action="store_true",
        help="Skip the initial safe-home move when the previous node already ended at HOME",
    )
    cli_args = rclpy.utilities.remove_ros_args(args=argv or sys.argv)[1:]
    return parser.parse_args(cli_args)


def run_corrective_branch(
    node,
    tray_name: str,
    ui_soil_status: str,
    requested_action: str,
    measurement_result: dict | None,
    *,
    skip_initial_home: bool = False,
) -> None:
    """Perform add/remove soil or rock removal using the latest probe result.

    A saved measurement avoids probing twice. The function still compares UI
    state with measured state, performs the selected physical route, then HOME.
    """
    apis = import_soil_service_apis(node)
    set_digital_outputs_fn, set_digital_output_fn = import_digital_output_apis()
    set_digital_outputs = set_outputs_compat(
        node,
        set_digital_outputs_fn=set_digital_outputs_fn,
        set_digital_output_fn=set_digital_output_fn,
    )

    if skip_initial_home:
        node.get_logger().info("Skip initial home: previous putdown already ended at HOME")
    else:
        go_home_with_gripper_held(node, apis)

    if measurement_result:
        result = measurement_result
        actual_state = result["soil_state"]
        node.get_logger().info("Use saved measurement result from dashboard state")
    else:
        # 저장 결과가 없으면 tray를 다시 읽어 상세 corner 데이터를 복원한다.
        result = inspect_tray_soil(node, apis, tray_name)
        actual_state = result["soil_state"]

    expected_state = SOIL_STATUS_TO_RESULT.get(ui_soil_status)
    if expected_state and actual_state != expected_state:
        node.get_logger().warn(
            f"UI soil_status={ui_soil_status} but result soil_state={actual_state}; "
            f"requested action={requested_action} 유지"
        )

    node.get_logger().info(
        f"Corrective action dispatch: requested={requested_action}, measured={actual_state}"
    )
    if requested_action == NEXT_ACTION_ADD_SOIL:
        run_soil_add_action(node, apis, set_digital_outputs, tray_name, result)
    elif requested_action == NEXT_ACTION_REMOVE_SOIL:
        run_soil_remove_action(node, apis, set_digital_outputs, tray_name, result)
    elif requested_action == NEXT_ACTION_REMOVE_ROCK:
        if require_rock_candidate(node, result):
            run_rock_remove_action(node, apis, set_digital_outputs, tray_name, result)
    else:
        raise ValueError(f"Unsupported corrective action: {requested_action}")

    go_home_with_gripper_held(node, apis)


def run_flatten_branch(node, tray_name: str) -> None:
    """Switch to press Tool/TCP, flatten one tray, HOME, then restore defaults."""
    apis = import_soil_service_apis(node)

    setup_tool_and_tcp(
        node, apis["set_tool"], apis["set_tcp"], TRAY_SOIL_TOOL_NAME, TRAY_SOIL_TCP_NAME
    )
    try:
        go_home_with_gripper_held(node, apis)
        run_tray_flatten_action(node, apis, tray_name)
        go_home_with_gripper_held(node, apis)
    finally:
        setup_tool_and_tcp(
            node, apis["set_tool"], apis["set_tcp"], COMMON_TOOL_NAME, COMMON_TCP_NAME
        )


def run_plant_branch(node, tray_label: str, plant_index: int) -> None:
    """Execute the taught plant source-to-selected-tray transfer sequence."""
    motion_apis = import_basic_motion_apis(node)
    set_digital_outputs_fn, set_digital_output_fn = import_digital_output_apis()
    set_digital_outputs = set_outputs_compat(
        node,
        set_digital_outputs_fn=set_digital_outputs_fn,
        set_digital_output_fn=set_digital_output_fn,
    )

    target_tray = TRAY_LABEL_TO_KEY[tray_label]
    node.get_logger().info(
        f"Selected source={PLANT_ACTIVE_TRAY}, plant={plant_index}, target={tray_label}"
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
        "integrated_tray_next_step plant complete; "
        f"final place joint={TARGET_TRAY_PLACE_JOINTS[target_tray]['down']}"
    )


def main(args=None):
    """Dispatch one dashboard next-step request to correction, flatten or plant."""
    parsed_args = parse_cli_args(args)
    rclpy.init(args=args)
    node = rclpy.create_node("integrated_tray_next_step", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    measurement_result = None
    if parsed_args.measurement_json:
        measurement_result = json.loads(parsed_args.measurement_json)

    try:
        if parsed_args.next_action in (
            NEXT_ACTION_ADD_SOIL,
            NEXT_ACTION_REMOVE_SOIL,
            NEXT_ACTION_REMOVE_ROCK,
        ):
            run_corrective_branch(
                node,
                parsed_args.tray,
                parsed_args.soil_status,
                parsed_args.next_action,
                measurement_result,
                skip_initial_home=parsed_args.skip_initial_home,
            )
        elif parsed_args.next_action == NEXT_ACTION_FLATTEN:
            run_flatten_branch(node, parsed_args.tray)
        elif parsed_args.next_action == NEXT_ACTION_PLANT:
            if parsed_args.plant is None:
                raise ValueError("--plant required for 식물 심기")
            run_plant_branch(node, parsed_args.tray, parsed_args.plant)
        elif parsed_args.next_action == NEXT_ACTION_MEASURE:
            node.get_logger().info("토양 측정은 dashboard run_measure 경로에서 처리")
        else:
            node.get_logger().info("완료 상태. 실행할 다음 작업 없음")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"integrated_tray_next_step failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
