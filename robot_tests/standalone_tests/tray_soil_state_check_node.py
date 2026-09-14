# 트레이 흙 상태를 확인하기 위해 내려가 접촉하고 다시 상승하는 체크 코드다.
from __future__ import annotations

import rclpy

from robot_tests.compliance_common import extract_fz, safe_release_compliance
from robot_tests.dsr_runtime import bootstrap_dsr_python
from robot_tests.motion_primitives import (
    HOME_JOINT,
    get_current_joint_timeout,
    is_home_joint,
    move_up_from_current,
    normalize_pose,
    set_outputs_compat,
)
from robot_tests.node_common import ROBOT_ID, ROBOT_MODEL, configure_dsr_init, import_digital_output_apis, setup_tool_and_tcp
from robot_tests.robot_motion_data import (
    TRAY_CORNERS_XYZ,
    TRAY_SOIL_ACC as ACC,
    TRAY_SOIL_CONTACT_DELTA_FZ as CONTACT_DELTA_FZ,
    TRAY_SOIL_HIGH_MAX as HIGH_MAX,
    TRAY_SOIL_HIGH_MIN as HIGH_MIN,
    TRAY_SOIL_LOW_MAX as LOW_MAX,
    TRAY_SOIL_LOW_MIN as LOW_MIN,
    TRAY_SOIL_MAX_DOWN_DISTANCE as MAX_DOWN_DISTANCE,
    TRAY_SOIL_OK_MAX as OK_MAX,
    TRAY_SOIL_OK_MIN as OK_MIN,
    TRAY_SOIL_OUTLIER_DIFF as OUTLIER_DIFF,
    TRAY_SOIL_PROBE_ACC as PROBE_ACC,
    TRAY_SOIL_PROBE_VEL as PROBE_VEL,
    TRAY_SOIL_TCP_NAME,
    TRAY_SOIL_TOOL_NAME,
    TRAY_SOIL_TOP_POSES,
    TRAY_SOIL_VEL as VEL,
)

bootstrap_dsr_python()

import DR_init

configure_dsr_init(DR_init)

def classify_single_z(z):
    if z is None:
        return "FAILED_CONTACT"
    if LOW_MIN <= z <= LOW_MAX:
        return "SOIL_LOW"
    if OK_MIN <= z <= OK_MAX:
        return "SOIL_OK"
    if HIGH_MIN <= z <= HIGH_MAX:
        return "SOIL_HIGH"
    return "UNKNOWN"


def describe_soil_state(state):
    if state == "SOIL_LOW":
        return "흙 부족"
    if state == "SOIL_OK":
        return "흙 적당"
    if state == "SOIL_HIGH":
        return "흙 많음"
    if state == "SKIPPED_DUE_TO_ROCK":
        return "돌맹이 있어 토양판정 생략"
    if state == "OBSTACLE_CANDIDATE":
        return "돌맹이 의심"
    if state == "FAILED_CONTACT":
        return "접촉 실패"
    return "판정 불가"


def classify_tray_result(corner_results):
    measured_corners = [dict(corner) for corner in corner_results if corner.get("z") is not None]
    failed_contacts = []

    for corner in corner_results:
        if corner.get("z") is None:
            failed_corner = dict(corner)
            failed_corner["state"] = "FAILED_CONTACT"
            failed_corner["diff_from_average"] = None
            failed_contacts.append(failed_corner)

    if measured_corners:
        avg_z_all = sum(corner["z"] for corner in measured_corners) / len(measured_corners)
    else:
        avg_z_all = None

    obstacle_candidates = []
    valid_corners = []

    for corner in measured_corners:
        z = corner["z"]
        diff = None if avg_z_all is None else z - avg_z_all

        corner_with_state = dict(corner)
        corner_with_state["diff_from_average"] = diff

        if diff is not None and abs(diff) >= OUTLIER_DIFF:
            corner_with_state["state"] = "OBSTACLE_CANDIDATE"
            obstacle_candidates.append(corner_with_state)
        else:
            corner_with_state["state"] = classify_single_z(z)
            valid_corners.append(corner_with_state)

    if not valid_corners:
        if failed_contacts:
            status = "CONTACT_FAILED"
        else:
            status = "FAILED_NO_VALID_SOIL_POINT"
        return {
            "soil_state": "UNKNOWN",
            "avg_z": None,
            "average_reference_z": avg_z_all,
            "obstacle_candidates": obstacle_candidates,
            "valid_corners": [],
            "failed_contacts": failed_contacts,
            "status": status,
        }

    if obstacle_candidates:
        avg_z = sum(corner["z"] for corner in measured_corners) / len(measured_corners)
        soil_state = "SKIPPED_DUE_TO_ROCK"
        status = "OBSTACLE_CANDIDATE_DETECTED"
    else:
        avg_z = sum(corner["z"] for corner in valid_corners) / len(valid_corners)

        if LOW_MIN <= avg_z <= LOW_MAX:
            soil_state = "SOIL_LOW"
        elif OK_MIN <= avg_z <= OK_MAX:
            soil_state = "SOIL_OK"
        elif HIGH_MIN <= avg_z <= HIGH_MAX:
            soil_state = "SOIL_HIGH"
        else:
            soil_state = "UNKNOWN"

    if not obstacle_candidates and failed_contacts:
        status = "CONTACT_FAILED"
    elif not obstacle_candidates and soil_state == "UNKNOWN":
        status = "UNKNOWN_SOIL_HEIGHT"
    elif not obstacle_candidates:
        status = "OK"

    return {
        "soil_state": soil_state,
        "avg_z": avg_z,
        "average_reference_z": avg_z_all,
        "obstacle_candidates": obstacle_candidates,
        "valid_corners": valid_corners,
        "failed_contacts": failed_contacts,
        "status": status,
    }


def log_classification_result(node, result):
    corner_states = {}
    for group_name in (
        "valid_corners",
        "obstacle_candidates",
        "failed_contacts",
    ):
        for corner in result[group_name]:
            corner_states[corner["name"]] = corner

    for corner_name in sorted(corner_states):
        corner = corner_states[corner_name]
        z = corner.get("z")
        z_text = "None" if z is None else f"{z:.3f}"
        state_desc = describe_soil_state(corner["state"])
        node.get_logger().info(
            f'{corner_name} z={z_text} state={corner["state"]} ({state_desc})'
        )

    average_reference_z = result["average_reference_z"]
    avg_z = result["avg_z"]
    average_ref_text = "None" if average_reference_z is None else f"{average_reference_z:.3f}"
    avg_text = "None" if avg_z is None else f"{avg_z:.3f}"

    soil_desc = describe_soil_state(result["soil_state"])
    node.get_logger().info(
        f"average_reference_z={average_ref_text} avg_z={avg_text} avg_judgement={soil_desc}"
    )
    node.get_logger().info(
        f'soil_state={result["soil_state"]} ({soil_desc}) status={result["status"]}'
    )

    for corner in result["obstacle_candidates"]:
        node.get_logger().warn(
            f'rock_suspected at {corner["name"]} x={corner["x"]:.3f} '
            f'y={corner["y"]:.3f} z={corner["z"]:.3f} '
            f'diff_from_average={corner["diff_from_average"]:.3f}'
        )

    for corner in result["failed_contacts"]:
        node.get_logger().warn(
            f'contact_failed at {corner["name"]} x={corner["x"]:.3f} '
            f'y={corner["y"]:.3f}'
        )


def go_home_with_gripper_held(
    node,
    set_digital_outputs,
    wait,
    get_current_posx,
    movel,
    movej,
    posx,
    posj,
    dr_base,
    get_current_posj,
) -> None:
    current_joint = get_current_joint_timeout(node)
    if is_home_joint(current_joint):
        node.get_logger().info(f"Safe HOME skip: robot is already at HOME -> {current_joint}")
        return

    node.get_logger().info("Step 1/2: move up before home while keeping gripper state")
    move_up_from_current(
        node=node,
        get_current_posx=get_current_posx,
        movel=movel,
        posx=posx,
        wait=wait,
        dr_base=dr_base,
    )

    node.get_logger().info(f"Step 2/2: move to home joint {HOME_JOINT}")
    movej(posj(HOME_JOINT), vel=60.0, acc=60.0)
    wait(0.5)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("tray_soil_state_check", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            DR_BASE,
            amovel,
            check_motion,
            get_current_posj,
            get_current_posx,
            get_tool_force,
            movej,
            movel,
            release_compliance_ctrl,
            set_tcp,
            set_tool,
            task_compliance_ctrl,
            wait,
        )
        from DR_common2 import posj, posx
    except ImportError as exc:
        node.get_logger().error(f"Failed to import Doosan API: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return

    set_digital_outputs_fn, set_digital_output_fn = import_digital_output_apis()

    def make_pose(x, y, z, rx, ry, rz):
        return posx(float(x), float(y), float(z), float(rx), float(ry), float(rz))

    def make_corner_top_pose(base_top_pose, x, y, z):
        # 트레이별 기본 상단 pose의 자세를 유지한 채 corner별 XYZ를 적용해 probing 시작 자세를 만든다.
        return make_pose(x, y, z, base_top_pose[3], base_top_pose[4], base_top_pose[5])

    def move_l(name, target, vel=VEL, acc=ACC, sleep=1.0):
        node.get_logger().info(f"Move start: {name} -> {target}")
        ret = movel(target, vel=vel, acc=acc)
        node.get_logger().info(f"Move ret: {name} = {ret}")
        wait(sleep)

    def return_to_pose(name, pose):
        node.get_logger().info(f"Return to {name}")
        movel(posx(pose), vel=PROBE_VEL, acc=PROBE_ACC)
        wait(1.0)

    def compliance_probe_down(tray_name, corner_name, top_pose):
        start_pose = normalize_pose(get_current_posx(ref=DR_BASE))
        start_z = start_pose[2]
        limit_z = start_z - MAX_DOWN_DISTANCE
        limit_pose = start_pose.copy()
        limit_pose[2] = limit_z

        node.get_logger().info(
            f"{tray_name}_{corner_name} compliance down start_z={start_z:.3f}, "
            f"limit_z={limit_z:.3f}, delta_fz={CONTACT_DELTA_FZ}"
        )

        task_compliance_ctrl()
        wait(0.5)
        base_force = get_tool_force(ref=DR_BASE)
        base_fz = extract_fz(base_force)
        node.get_logger().info(f"{tray_name}_{corner_name} base_fz={base_fz:.3f}")

        contact_detected = False
        current_z = start_z

        amovel(posx(limit_pose), vel=PROBE_VEL, acc=PROBE_ACC)

        while check_motion() == 2:
            wait(0.05)
            current_pose = normalize_pose(get_current_posx(ref=DR_BASE))
            current_z = current_pose[2]
            current_fz = extract_fz(get_tool_force(ref=DR_BASE))
            delta_fz = abs(current_fz - base_fz)

            if delta_fz >= CONTACT_DELTA_FZ:
                node.get_logger().info(
                    f"{tray_name}_{corner_name} contact detected z={current_z:.3f}"
                )
                contact_detected = True
                break

        if not contact_detected:
            current_pose = normalize_pose(get_current_posx(ref=DR_BASE))
            current_z = current_pose[2]
            node.get_logger().warn(
                f"{tray_name}_{corner_name} contact failed, limit reached z={current_z:.3f}"
            )

        safe_release_compliance(node, release_compliance_ctrl)
        return_to_pose(f"{tray_name}_{corner_name}_TOP", top_pose)

        if not contact_detected:
            return None
        return current_z

    def probe_corner(tray_name, corner_name, x, y, z, tray_top_pose):
        top_pose = make_corner_top_pose(tray_top_pose, x, y, z)

        move_l(
            f"{tray_name}_{corner_name}_TOP",
            top_pose,
            vel=VEL,
            acc=ACC,
            sleep=0.5,
        )

        contact_z = compliance_probe_down(tray_name, corner_name, top_pose)
        return {
            "name": corner_name,
            "x": float(x),
            "y": float(y),
            "z": contact_z,
        }

    def inspect_tray(tray_name):
        tray_top_pose = normalize_pose(TRAY_SOIL_TOP_POSES[tray_name])
        corners = TRAY_CORNERS_XYZ[tray_name]
        corner_results = []

        move_l(f"{tray_name}_CENTER_TOP", tray_top_pose)

        for corner_name, xyz in corners:
            x, y, z = xyz
            corner_result = probe_corner(tray_name, corner_name, x, y, z, tray_top_pose)
            corner_results.append(corner_result)

        result = classify_tray_result(corner_results)
        log_classification_result(node, result)

        move_l(f"{tray_name}_CENTER_TOP_RETURN", tray_top_pose)
        return result

    node.get_logger().info("tray_soil_state_check started")

    try:
        set_digital_outputs = set_outputs_compat(
            node,
            set_digital_outputs_fn=set_digital_outputs_fn,
            set_digital_output_fn=set_digital_output_fn,
        )

        setup_tool_and_tcp(node, set_tool, set_tcp, TRAY_SOIL_TOOL_NAME, TRAY_SOIL_TCP_NAME)

        wait(0.5)

        go_home_with_gripper_held(
            node=node,
            set_digital_outputs=set_digital_outputs,
            wait=wait,
            get_current_posx=get_current_posx,
            movel=movel,
            movej=movej,
            posx=posx,
            posj=posj,
            dr_base=DR_BASE,
            get_current_posj=get_current_posj,
        )

        while rclpy.ok():
            cmd = input("\n트레이 선택 입력 [A/B/C/D], 종료 [q]: ").strip().upper()

            if cmd == "Q":
                node.get_logger().info("Quit command received")
                break

            if cmd not in TRAY_SOIL_TOP_POSES:
                print("잘못된 입력입니다. A, B, C, D, q 중 하나를 입력하세요.")
                continue

            node.get_logger().info(f"Selected tray: {cmd}")
            inspect_tray(cmd)

        go_home_with_gripper_held(
            node=node,
            set_digital_outputs=set_digital_outputs,
            wait=wait,
            get_current_posx=get_current_posx,
            movel=movel,
            movej=movej,
            posx=posx,
            posj=posj,
            dr_base=DR_BASE,
            get_current_posj=get_current_posj,
        )
        node.get_logger().info("tray_soil_state_check finished")

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as exc:
        node.get_logger().error(f"tray_soil_state_check failed: {exc}")
        safe_release_compliance(node, release_compliance_ctrl)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
