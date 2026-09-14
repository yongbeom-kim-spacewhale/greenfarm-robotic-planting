"""Common helpers for tray soil service action nodes."""

from __future__ import annotations

from robot_tests.compliance_common import extract_fz, safe_release_compliance
from robot_tests.motion_primitives import (
    GRIPPER_HOME_PREPARE,
    GRIPPER_PICK_CLOSE,
    HOME_JOINT,
    gripper_command,
    get_current_joint_timeout,
    is_home_joint,
    move_j,
    move_l,
    move_up_from_current,
    normalize_pose,
    prepare_home_joint,
)
from robot_tests.robot_motion_data import (
    A_TRAY_SOIL_DROP_CYCLES,
    A_TRAY_SOIL_DROP_PREP_JOINTS,
    A_TRAY_SOIL_DROP_TOP_JOINT,
    B_TRAY_SOIL_DROP_CYCLES,
    B_TRAY_SOIL_DROP_PREP_JOINTS,
    B_TRAY_SOIL_DROP_TOP_JOINT,
    C_TRAY_REMOVE_APPROACH_JOINT,
    C_TRAY_REMOVE_BOX_DUMP_JOINT,
    C_TRAY_REMOVE_BOX_TOP_JOINT,
    C_TRAY_REMOVE_POSTURE_ADJUST_JOINT,
    C_TRAY_REMOVE_SCOOP_END_POSE,
    C_TRAY_REMOVE_SCOOP_START_JOINT,
    C_TRAY_REMOVE_SCOOP_START_POSE,
    C_TRAY_REMOVE_SHAKE_STEPS,
    C_TRAY_REMOVE_SHAKE_Z_AMPLITUDE,
    C_TRAY_SOIL_DROP_TOP_JOINT,
    D_TRAY_SOIL_DROP_CYCLES,
    D_TRAY_SOIL_DROP_PREP_JOINTS,
    D_TRAY_SOIL_DROP_TOP_JOINT,
    ROCK_REMOVE_LIFT_POSE,
    ROCK_REMOVE_PICK_POSE,
    ROCK_REMOVE_READY_JOINT,
    ROCK_REMOVE_WASTE_DROP_POSE,
    ROCK_REMOVE_WASTE_TOP_POSE,
    SHOVEL_PICK_JOINT,
    SHOVEL_READY_JOINT,
    SOIL_DROP_C_TRAY_CYCLES,
    SOIL_SUPPLY_BOX_SCOOP_POSE,
    SOIL_SUPPLY_BOX_TOP_POSE,
    SOIL_SUPPLY_JOINT_SEQUENCE,
    TRAY_CORNERS_XYZ,
    TRAY_SOIL_ACC,
    TRAY_SOIL_CONTACT_DELTA_FZ,
    TRAY_SOIL_FLATTEN_ACC,
    TRAY_SOIL_FLATTEN_CONTACT_DELTA_FZ,
    TRAY_SOIL_FLATTEN_EXTRA_PRESS_MM,
    TRAY_SOIL_FLATTEN_HOLD_SEC,
    TRAY_SOIL_FLATTEN_VEL,
    TRAY_SOIL_MAX_DOWN_DISTANCE,
    TRAY_SOIL_PROBE_ACC,
    TRAY_SOIL_PROBE_VEL,
    TRAY_SOIL_REMOVE_LIFT_Z_OFFSET,
    TRAY_SOIL_REMOVE_SHAKE_AMP,
    TRAY_SOIL_REMOVE_SHAKE_ATIME,
    TRAY_SOIL_REMOVE_SHAKE_PERIOD,
    TRAY_SOIL_REMOVE_SHAKE_REPEAT,
    TRAY_SOIL_REMOVE_SCOOP_Z_OFFSET,
    TRAY_SOIL_REMOVE_TARGET_NAMES,
    TRAY_SOIL_TOP_POSES,
    TRAY_SOIL_VEL,
)
from robot_tests.standalone_tests.tray_soil_state_check_node import (
    classify_tray_result,
    describe_soil_state,
    log_classification_result,
)


def prompt_choice(prompt: str, valid_values: list[str]) -> str:
    """Read a validated operator choice for standalone test nodes."""
    valid_map = {value.upper(): value for value in valid_values}
    while True:
        raw = input(prompt).strip()
        if not raw:
            print("입력이 비었습니다. 다시 입력하세요.")
            continue
        if raw.lower() == "q":
            raise KeyboardInterrupt
        key = raw.upper()
        if key not in valid_map:
            print(f"잘못된 입력입니다. 가능한 값: {', '.join(valid_values)}")
            continue
        return valid_map[key]


def prompt_target_tray() -> str:
    """Ask a standalone operator which physical tray to service."""
    return prompt_choice("\n작업할 트레이 선택 [A/B/C/D], 종료 [q]: ", ["A", "B", "C", "D"]).upper()


def import_soil_service_apis(node):
    """Load motion, force and compliance SDK calls required by soil workflows."""
    try:
        from DSR_ROBOT2 import (
            DR_BASE,
            DR_TOOL,
            amovel,
            check_motion,
            get_current_posj,
            get_current_posx,
            get_tool_force,
            move_periodic,
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
        raise

    return {
        "DR_BASE": DR_BASE,
        "DR_TOOL": DR_TOOL,
        "amovel": amovel,
        "check_motion": check_motion,
        "get_current_posj": get_current_posj,
        "get_current_posx": get_current_posx,
        "get_tool_force": get_tool_force,
        "move_periodic": move_periodic,
        "movej": movej,
        "movel": movel,
        "posj": posj,
        "posx": posx,
        "release_compliance_ctrl": release_compliance_ctrl,
        "set_tcp": set_tcp,
        "set_tool": set_tool,
        "task_compliance_ctrl": task_compliance_ctrl,
        "wait": wait,
    }


def make_pose(posx_fn, x: float, y: float, z: float, rx: float, ry: float, rz: float):
    """Build a Doosan Cartesian pose with explicit floating-point values."""
    return posx_fn(float(x), float(y), float(z), float(rx), float(ry), float(rz))


def make_tray_pose(posx_fn, tray_name: str, x: float, y: float, z: float):
    """Build a tray point while preserving that tray's taught tool orientation."""
    tray_top_pose = normalize_pose(TRAY_SOIL_TOP_POSES[tray_name])
    return make_pose(posx_fn, x, y, z, tray_top_pose[3], tray_top_pose[4], tray_top_pose[5])


def corner_xyz_map(tray_name: str) -> dict[str, tuple[float, float, float]]:
    """Expose named measurement corners for one tray as a lookup table."""
    return {corner_name: xyz for corner_name, xyz in TRAY_CORNERS_XYZ[tray_name]}


def build_corner_pose(posx_fn, tray_name: str, corner_name: str, target_z: float | None = None):
    """Create an approach/contact pose for a named tray corner."""
    x, y, default_z = corner_xyz_map(tray_name)[corner_name]
    z = default_z if target_z is None else target_z
    return make_tray_pose(posx_fn, tray_name, x, y, z)


def build_tray_center_pose(
    posx_fn,
    tray_name: str,
    z_offset: float = 0.0,
):
    """Create a centre pose offset vertically from the tray's taught top pose."""
    tray_top_pose = normalize_pose(TRAY_SOIL_TOP_POSES[tray_name])
    target_pose = tray_top_pose.copy()
    target_pose[2] += float(z_offset)
    return posx_fn(target_pose)


def go_home_with_gripper_held(node, apis, *, move_up: bool = True) -> None:
    """Return HOME without changing the currently held tool's gripper state."""
    try:
        current_joint = get_current_joint_timeout(node)
        if is_home_joint(current_joint):
            node.get_logger().info(
                f"Safe HOME skip: robot is already at HOME -> {current_joint}"
            )
            return
    except Exception as exc:
        node.get_logger().warn(
            f"HOME state check failed; execute safe HOME sequence: {exc}"
        )

    if move_up:
        node.get_logger().info("Move up before home while keeping current gripper state")
        current_pose = normalize_pose(apis["get_current_posx"](ref=apis["DR_BASE"]))
        target_pose = current_pose.copy()
        target_pose[2] += 120.0
        apis["movel"](apis["posx"](target_pose), vel=60.0, acc=60.0)
        apis["wait"](0.5)
    else:
        node.get_logger().info("Skip extra Z lift; current pose is already at safe top")

    node.get_logger().info(f"Move to home joint {HOME_JOINT}")
    apis["movej"](apis["posj"](HOME_JOINT), vel=60.0, acc=60.0)
    apis["wait"](0.5)


def inspect_tray_soil(node, apis, tray_name: str) -> dict:
    """Probe four corners with compliance control and classify the soil state.

    Each corner descends asynchronously until force change indicates contact or
    the Z travel limit is reached, then returns to its top pose. The returned
    dictionary is persisted by the dashboard and drives the next operation.
    """
    def move_line(name: str, target, vel: float = TRAY_SOIL_VEL, acc: float = TRAY_SOIL_ACC, sleep: float = 0.5):
        """Move linearly to a named probe waypoint and log the SDK result."""
        node.get_logger().info(f"Move start: {name} -> {target}")
        ret = apis["movel"](target, vel=vel, acc=acc)
        node.get_logger().info(f"Move ret: {name} = {ret}")
        apis["wait"](sleep)

    def return_to_pose(name: str, pose_values: list[float]) -> None:
        """Retract to a saved top pose after probing a corner."""
        node.get_logger().info(f"Return to {name}")
        apis["movel"](apis["posx"](pose_values), vel=TRAY_SOIL_PROBE_VEL, acc=TRAY_SOIL_PROBE_ACC)
        apis["wait"](0.5)

    def compliance_probe_down(corner_name: str, top_pose_values: list[float]):
        """Measure one corner contact height using force-change detection."""
        start_pose = normalize_pose(apis["get_current_posx"](ref=apis["DR_BASE"]))
        start_z = start_pose[2]
        limit_z = start_z - TRAY_SOIL_MAX_DOWN_DISTANCE
        limit_pose = start_pose.copy()
        limit_pose[2] = limit_z

        node.get_logger().info(
            f"{tray_name}_{corner_name} compliance down start_z={start_z:.3f}, "
            f"limit_z={limit_z:.3f}, delta_fz={TRAY_SOIL_CONTACT_DELTA_FZ}"
        )

        apis["task_compliance_ctrl"]()
        apis["wait"](0.5)
        base_fz = extract_fz(apis["get_tool_force"](ref=apis["DR_BASE"]))
        node.get_logger().info(f"{tray_name}_{corner_name} base_fz={base_fz:.3f}")

        contact_detected = False
        current_z = start_z

        apis["amovel"](apis["posx"](limit_pose), vel=TRAY_SOIL_PROBE_VEL, acc=TRAY_SOIL_PROBE_ACC)

        while apis["check_motion"]() == 2:
            apis["wait"](0.05)
            current_pose = normalize_pose(apis["get_current_posx"](ref=apis["DR_BASE"]))
            current_z = current_pose[2]
            current_fz = extract_fz(apis["get_tool_force"](ref=apis["DR_BASE"]))
            delta_fz = abs(current_fz - base_fz)

            if delta_fz >= TRAY_SOIL_CONTACT_DELTA_FZ:
                node.get_logger().info(
                    f"{tray_name}_{corner_name} contact detected z={current_z:.3f}"
                )
                contact_detected = True
                break

        if not contact_detected:
            current_pose = normalize_pose(apis["get_current_posx"](ref=apis["DR_BASE"]))
            current_z = current_pose[2]
            node.get_logger().warn(
                f"{tray_name}_{corner_name} contact failed, limit reached z={current_z:.3f}"
            )

        safe_release_compliance(node, apis["release_compliance_ctrl"])
        return_to_pose(f"{tray_name}_{corner_name}_TOP", top_pose_values)

        if not contact_detected:
            return None
        return current_z

    tray_top_pose = normalize_pose(TRAY_SOIL_TOP_POSES[tray_name])
    move_line(f"{tray_name}_CENTER_TOP", apis["posx"](tray_top_pose))

    corner_results = []
    for corner_name, (x, y, z) in TRAY_CORNERS_XYZ[tray_name]:
        top_pose = make_tray_pose(apis["posx"], tray_name, x, y, z)
        move_line(
            f"{tray_name}_{corner_name}_TOP",
            top_pose,
            vel=TRAY_SOIL_VEL,
            acc=TRAY_SOIL_ACC,
            sleep=0.5,
        )
        contact_z = compliance_probe_down(corner_name, normalize_pose(top_pose))
        corner_results.append(
            {
                "name": corner_name,
                "x": float(x),
                "y": float(y),
                "z": contact_z,
            }
        )

    result = classify_tray_result(corner_results)
    log_classification_result(node, result)
    move_line(f"{tray_name}_CENTER_TOP_RETURN", apis["posx"](tray_top_pose))
    return result


def require_soil_state(node, result: dict, expected_state: str) -> bool:
    """Guard an action that is valid only for one classified soil state."""
    actual = result["soil_state"]
    if actual != expected_state:
        node.get_logger().warn(
            f"Expected {expected_state}, but tray result is {actual} ({describe_soil_state(actual)})"
        )
        return False
    return True


def require_rock_candidate(node, result: dict) -> bool:
    """Confirm probe data contains the obstacle signature required for removal."""
    candidates = result["obstacle_candidates"]
    if not candidates:
        node.get_logger().warn("No rock candidate found. Nothing to remove.")
        return False
    return True


def pickup_shovel(node, apis, set_digital_outputs) -> None:
    """Follow the taught rack route and close the gripper on the shovel."""
    node.get_logger().info("Pickup shovel start")
    # 자동 흙 작업도 대시보드의 툴 상태 머신과 같은 단계 표식을 사용한다.
    node.get_logger().info("tool_phase=SHOVEL_PICKING_APPROACH")
    prepare_home_joint(node, set_digital_outputs, apis["wait"])
    move_j(node, apis["movej"], apis["posj"], apis["wait"], "HOME", HOME_JOINT)
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "SHOVEL_READY",
        SHOVEL_READY_JOINT,
        vel=80.0,
        acc=80.0,
        check_motion=apis["check_motion"],
    )
    gripper_command(
        node,
        set_digital_outputs,
        apis["wait"],
        "SHOVEL_OPEN_0100",
        GRIPPER_HOME_PREPARE,
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "SHOVEL_PICK",
        SHOVEL_PICK_JOINT,
        vel=20.0,
        acc=20.0,
        check_motion=apis["check_motion"],
    )
    node.get_logger().info("tool_phase=SHOVEL_PICKING_GRIP")
    gripper_command(
        node,
        set_digital_outputs,
        apis["wait"],
        "SHOVEL_GRIP_0010",
        GRIPPER_PICK_CLOSE,
    )
    node.get_logger().info("tool_phase=SHOVEL_HELD")
    move_up_from_current(
        node=node,
        get_current_posx=apis["get_current_posx"],
        movel=apis["movel"],
        posx=apis["posx"],
        wait=apis["wait"],
        dr_base=apis["DR_BASE"],
        z_offset=120.0,
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "HOME_RETURN",
        HOME_JOINT,
        vel=80.0,
        acc=80.0,
        check_motion=apis["check_motion"],
    )


def return_shovel(node, apis, set_digital_outputs) -> None:
    """Return the shovel to its rack and leave the gripper in HOME preparation."""
    node.get_logger().info("Return shovel start")
    node.get_logger().info("tool_phase=SHOVEL_PUTTING_APPROACH")
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "HOME",
        HOME_JOINT,
        vel=80.0,
        acc=80.0,
        check_motion=apis["check_motion"],
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "SHOVEL_READY",
        SHOVEL_READY_JOINT,
        vel=80.0,
        acc=80.0,
        check_motion=apis["check_motion"],
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "SHOVEL_RETURN_DOWN",
        SHOVEL_PICK_JOINT,
        vel=20.0,
        acc=20.0,
        check_motion=apis["check_motion"],
    )
    node.get_logger().info("tool_phase=SHOVEL_PUTTING_RELEASE")
    gripper_command(
        node,
        set_digital_outputs,
        apis["wait"],
        "SHOVEL_RELEASE_0100",
        GRIPPER_HOME_PREPARE,
    )
    node.get_logger().info("tool_phase=SHOVEL_RELEASED")
    prepare_home_joint(node, set_digital_outputs, apis["wait"])
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "HOME_RETURN",
        HOME_JOINT,
        vel=80.0,
        acc=80.0,
        check_motion=apis["check_motion"],
    )


def select_target_corners(tray_name: str, result: dict, target_names: tuple[str, ...]) -> list[dict]:
    """Choose correction corners from valid probe heights, with taught fallbacks."""
    corner_lookup = {corner["name"]: dict(corner) for corner in result.get("valid_corners", [])}
    xyz_lookup = corner_xyz_map(tray_name)
    avg_z = result.get("avg_z")
    selected = []

    for name in target_names:
        if name in corner_lookup:
            selected.append(corner_lookup[name])
            continue

        x, y, default_z = xyz_lookup[name]
        selected.append(
            {
                "name": name,
                "x": float(x),
                "y": float(y),
                "z": default_z if avg_z is None else avg_z,
            }
        )
    return selected


def fill_shovel_from_supply(node, apis) -> None:
    """Move the held shovel through the taught soil-supply scoop path."""
    node.get_logger().info("Run soil supply joint sequence")
    for index, joint_values in enumerate(SOIL_SUPPLY_JOINT_SEQUENCE, start=1):
        move_j(
            node,
            apis["movej"],
            apis["posj"],
            apis["wait"],
            f"SOIL_SUPPLY_STEP_{index}",
            joint_values,
            vel=20.0,
            acc=20.0,
        )


def dump_to_waste_box(node, apis) -> None:
    """Carry removed soil to the waste box, tip it out, and retreat."""
    """Dump removed soil through the taught C-tray waste-box joint route."""
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "WASTE_BOX_TOP_JOINT",
        C_TRAY_REMOVE_BOX_TOP_JOINT,
        vel=40.0,
        acc=40.0,
        check_motion=apis["check_motion"],
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "WASTE_BOX_DUMP_JOINT",
        C_TRAY_REMOVE_BOX_DUMP_JOINT,
        vel=20.0,
        acc=20.0,
        check_motion=apis["check_motion"],
    )
    apis["wait"](1.0)
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "WASTE_BOX_TOP_RETURN_JOINT",
        C_TRAY_REMOVE_BOX_TOP_JOINT,
        vel=40.0,
        acc=40.0,
        check_motion=apis["check_motion"],
    )


def run_tray_flatten_action(node, apis, tray_name: str) -> dict:
    """Compliance-press tray corners to level soil and report press results."""
    """Flatten all four tray corners using force-based contact detection."""
    node.get_logger().info(f"Run compliance tray flatten action tray={tray_name}")
    tray_top_pose = normalize_pose(TRAY_SOIL_TOP_POSES[tray_name])
    move_l(
        node,
        apis["movel"],
        apis["posx"],
        apis["wait"],
        f"{tray_name}_CENTER_TOP",
        tray_top_pose,
        vel=TRAY_SOIL_VEL,
        acc=TRAY_SOIL_ACC,
    )

    corner_results = []
    for corner_name, (x, y, z) in TRAY_CORNERS_XYZ[tray_name]:
        top_pose = normalize_pose(make_tray_pose(apis["posx"], tray_name, x, y, z))
        move_l(
            node,
            apis["movel"],
            apis["posx"],
            apis["wait"],
            f"{tray_name}_{corner_name}_FLATTEN_TOP",
            top_pose,
            vel=TRAY_SOIL_VEL,
            acc=TRAY_SOIL_ACC,
        )

        start_pose = normalize_pose(apis["get_current_posx"](ref=apis["DR_BASE"]))
        limit_pose = start_pose.copy()
        limit_pose[2] -= TRAY_SOIL_MAX_DOWN_DISTANCE
        node.get_logger().info(
            f"{tray_name}_{corner_name} flatten down start_z={start_pose[2]:.3f}, "
            f"delta_fz={TRAY_SOIL_FLATTEN_CONTACT_DELTA_FZ}"
        )

        contact_pose = None
        try:
            apis["task_compliance_ctrl"]()
            apis["wait"](0.5)
            base_fz = extract_fz(apis["get_tool_force"](ref=apis["DR_BASE"]))
            apis["amovel"](
                apis["posx"](limit_pose),
                vel=TRAY_SOIL_PROBE_VEL,
                acc=TRAY_SOIL_PROBE_ACC,
            )

            while apis["check_motion"]() == 2:
                apis["wait"](0.05)
                current_pose = normalize_pose(
                    apis["get_current_posx"](ref=apis["DR_BASE"])
                )
                current_fz = extract_fz(
                    apis["get_tool_force"](ref=apis["DR_BASE"])
                )
                if abs(current_fz - base_fz) >= TRAY_SOIL_FLATTEN_CONTACT_DELTA_FZ:
                    contact_pose = current_pose
                    node.get_logger().info(
                        f"{tray_name}_{corner_name} flatten contact "
                        f"z={current_pose[2]:.3f}"
                    )
                    break
        finally:
            safe_release_compliance(node, apis["release_compliance_ctrl"])

        if contact_pose is None:
            node.get_logger().warn(
                f"{tray_name}_{corner_name} flatten contact failed; skip extra press"
            )
        else:
            press_pose = contact_pose.copy()
            press_pose[2] -= TRAY_SOIL_FLATTEN_EXTRA_PRESS_MM
            move_l(
                node,
                apis["movel"],
                apis["posx"],
                apis["wait"],
                f"{tray_name}_{corner_name}_PRESS_EXTRA",
                press_pose,
                vel=TRAY_SOIL_FLATTEN_VEL,
                acc=TRAY_SOIL_FLATTEN_ACC,
            )
            apis["wait"](TRAY_SOIL_FLATTEN_HOLD_SEC)

        corner_results.append(
            {
                "name": corner_name,
                "x": float(x),
                "y": float(y),
                "z": None if contact_pose is None else float(contact_pose[2]),
            }
        )

        move_l(
            node,
            apis["movel"],
            apis["posx"],
            apis["wait"],
            f"{tray_name}_{corner_name}_FLATTEN_TOP_RETURN",
            top_pose,
            vel=TRAY_SOIL_PROBE_VEL,
            acc=TRAY_SOIL_PROBE_ACC,
        )

    move_l(
        node,
        apis["movel"],
        apis["posx"],
        apis["wait"],
        f"{tray_name}_CENTER_TOP_RETURN",
        tray_top_pose,
        vel=TRAY_SOIL_VEL,
        acc=TRAY_SOIL_ACC,
    )
    result = classify_tray_result(corner_results)
    log_classification_result(node, result)
    return result


def shake_scoop_in_tray(node, apis) -> None:
    """Oscillate the shovel during a soil-removal scoop to loosen material."""
    move_periodic = apis.get("move_periodic")
    if move_periodic is not None:
        node.get_logger().info(
            "Run tray scoop shake with move_periodic "
            f"amp={TRAY_SOIL_REMOVE_SHAKE_AMP} period={TRAY_SOIL_REMOVE_SHAKE_PERIOD} "
            f"atime={TRAY_SOIL_REMOVE_SHAKE_ATIME} repeat={TRAY_SOIL_REMOVE_SHAKE_REPEAT}"
        )
        ret = move_periodic(
            TRAY_SOIL_REMOVE_SHAKE_AMP,
            TRAY_SOIL_REMOVE_SHAKE_PERIOD,
            atime=TRAY_SOIL_REMOVE_SHAKE_ATIME,
            repeat=TRAY_SOIL_REMOVE_SHAKE_REPEAT,
            ref=apis["DR_TOOL"],
        )
        node.get_logger().info(f"move_periodic ret={ret}")
        apis["wait"](0.5)
        return

    node.get_logger().warn("move_periodic API unavailable, use movel shake fallback")
    current_pose = normalize_pose(apis["get_current_posx"](ref=apis["DR_BASE"]))
    x_amp = float(TRAY_SOIL_REMOVE_SHAKE_AMP[0])
    y_amp = float(TRAY_SOIL_REMOVE_SHAKE_AMP[1])

    offsets = [
        (x_amp, 0.0),
        (-x_amp, 0.0),
        (0.0, y_amp),
        (0.0, -y_amp),
    ]
    for repeat_index in range(TRAY_SOIL_REMOVE_SHAKE_REPEAT):
        for offset_x, offset_y in offsets:
            target_pose = current_pose.copy()
            target_pose[0] += offset_x
            target_pose[1] += offset_y
            move_l(
                node,
                apis["movel"],
                apis["posx"],
                apis["wait"],
                f"TRAY_SHAKE_{repeat_index + 1}",
                target_pose,
                vel=TRAY_SOIL_PROBE_VEL,
                acc=TRAY_SOIL_PROBE_ACC,
            )
    move_l(
        node,
        apis["movel"],
        apis["posx"],
        apis["wait"],
        "TRAY_SHAKE_RETURN",
        current_pose,
        vel=TRAY_SOIL_PROBE_VEL,
        acc=TRAY_SOIL_PROBE_ACC,
    )


def lerp_pose(start_pose: list[float], end_pose: list[float], ratio: float) -> list[float]:
    """Interpolate a Cartesian waypoint used by the custom C-tray scoop route."""
    return [
        float(start_value + ((end_value - start_value) * ratio))
        for start_value, end_value in zip(start_pose, end_pose)
    ]


def run_c_tray_custom_remove_action(node, apis) -> None:
    """Execute C tray's dedicated taught scoop, shake, lift and dump sequence."""
    node.get_logger().info("Use taught C-tray scoop path. Assumption: shovel is already gripped.")
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "C_TRAY_REMOVE_APPROACH",
        C_TRAY_REMOVE_APPROACH_JOINT,
        vel=40.0,
        acc=40.0,
        check_motion=apis["check_motion"],
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "C_TRAY_REMOVE_SCOOP_START",
        C_TRAY_REMOVE_SCOOP_START_JOINT,
        vel=25.0,
        acc=25.0,
        check_motion=apis["check_motion"],
    )

    for step_index in range(1, C_TRAY_REMOVE_SHAKE_STEPS + 1):
        ratio = step_index / C_TRAY_REMOVE_SHAKE_STEPS
        target_pose = lerp_pose(C_TRAY_REMOVE_SCOOP_START_POSE, C_TRAY_REMOVE_SCOOP_END_POSE, ratio)
        target_pose[2] += C_TRAY_REMOVE_SHAKE_Z_AMPLITUDE if step_index % 2 == 1 else -C_TRAY_REMOVE_SHAKE_Z_AMPLITUDE
        move_l(
            node,
            apis["movel"],
            apis["posx"],
            apis["wait"],
            f"C_TRAY_SHAKE_STEP_{step_index}",
            target_pose,
            vel=TRAY_SOIL_PROBE_VEL,
            acc=TRAY_SOIL_PROBE_ACC,
        )

    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "C_TRAY_POSTURE_ADJUST",
        C_TRAY_REMOVE_POSTURE_ADJUST_JOINT,
        vel=40.0,
        acc=40.0,
        check_motion=apis["check_motion"],
    )

    current_pose = normalize_pose(apis["get_current_posx"](ref=apis["DR_BASE"]))
    lift_pose = current_pose.copy()
    lift_pose[2] += 30.0
    move_l(
        node,
        apis["movel"],
        apis["posx"],
        apis["wait"],
        "C_TRAY_SCOOP_LIFT",
        lift_pose,
        vel=TRAY_SOIL_PROBE_VEL,
        acc=TRAY_SOIL_PROBE_ACC,
    )

    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "C_TRAY_BOX_TOP",
        C_TRAY_REMOVE_BOX_TOP_JOINT,
        vel=40.0,
        acc=40.0,
        check_motion=apis["check_motion"],
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "C_TRAY_BOX_DUMP",
        C_TRAY_REMOVE_BOX_DUMP_JOINT,
        vel=20.0,
        acc=20.0,
        check_motion=apis["check_motion"],
    )
    apis["wait"](1.0)
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "C_TRAY_BOX_TOP_RETURN",
        C_TRAY_REMOVE_BOX_TOP_JOINT,
        vel=40.0,
        acc=40.0,
        check_motion=apis["check_motion"],
    )


def _drop_soil_to_named_tray_test(
    node,
    apis,
    *,
    tray_label: str,
    top_joint: list[float],
    cycles: list[dict],
    cycle_index: int,
    prep_joints: list[list[float]] | None = None,
) -> None:
    """Execute one indexed taught soil-drop cycle for the selected tray."""
    cycle = cycles[cycle_index]
    human_index = cycle_index + 1
    total_cycles = len(cycles)
    node.get_logger().info(f"Run {tray_label} tray soil drop test cycle {human_index}/{total_cycles}")
    if prep_joints:
        for prep_index, prep_joint in enumerate(prep_joints, start=1):
            move_j(
                node,
                apis["movej"],
                apis["posj"],
                apis["wait"],
                f"{tray_label}_TRAY_PREP_{human_index}_{prep_index}",
                prep_joint,
                vel=40.0,
                acc=40.0,
                check_motion=apis["check_motion"],
            )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        f"{tray_label}_TRAY_DROP_TOP",
        top_joint,
        vel=40.0,
        acc=40.0,
        check_motion=apis["check_motion"],
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        f"{tray_label}_TRAY_DROP_{human_index}_APPROACH",
        cycle["approach"],
        vel=20.0,
        acc=20.0,
        check_motion=apis["check_motion"],
    )
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        f"{tray_label}_TRAY_DROP_{human_index}_DROP",
        cycle["drop"],
        vel=20.0,
        acc=20.0,
        check_motion=apis["check_motion"],
    )


def drop_soil_to_a_tray_test(node, apis, cycle_index: int) -> None:
    """Run one indexed taught soil-drop route for tray A."""
    _drop_soil_to_named_tray_test(
        node,
        apis,
        tray_label="A",
        top_joint=A_TRAY_SOIL_DROP_TOP_JOINT,
        cycles=A_TRAY_SOIL_DROP_CYCLES,
        cycle_index=cycle_index,
        prep_joints=A_TRAY_SOIL_DROP_PREP_JOINTS,
    )


def drop_soil_to_c_tray_test(node, apis, cycle_index: int) -> None:
    """Run one indexed taught soil-drop route for tray C."""
    _drop_soil_to_named_tray_test(
        node,
        apis,
        tray_label="C",
        top_joint=C_TRAY_SOIL_DROP_TOP_JOINT,
        cycles=SOIL_DROP_C_TRAY_CYCLES,
        cycle_index=cycle_index,
    )


def drop_soil_to_b_tray_test(node, apis, cycle_index: int) -> None:
    """Run one indexed taught soil-drop route for tray B."""
    _drop_soil_to_named_tray_test(
        node,
        apis,
        tray_label="B",
        top_joint=B_TRAY_SOIL_DROP_TOP_JOINT,
        cycles=B_TRAY_SOIL_DROP_CYCLES,
        cycle_index=cycle_index,
        prep_joints=B_TRAY_SOIL_DROP_PREP_JOINTS,
    )


def drop_soil_to_d_tray_test(node, apis, cycle_index: int) -> None:
    """Run one indexed taught soil-drop route for tray D."""
    _drop_soil_to_named_tray_test(
        node,
        apis,
        tray_label="D",
        top_joint=D_TRAY_SOIL_DROP_TOP_JOINT,
        cycles=D_TRAY_SOIL_DROP_CYCLES,
        cycle_index=cycle_index,
        prep_joints=D_TRAY_SOIL_DROP_PREP_JOINTS,
    )


def run_soil_add_action(node, apis, set_digital_outputs, tray_name: str, result: dict) -> None:
    """Pick shovel, repeat supply-to-tray drops, then return the shovel."""
    pickup_shovel(node, apis, set_digital_outputs)

    drop_plans = {
        "A": (A_TRAY_SOIL_DROP_CYCLES, drop_soil_to_a_tray_test),
        "B": (B_TRAY_SOIL_DROP_CYCLES, drop_soil_to_b_tray_test),
        "C": (SOIL_DROP_C_TRAY_CYCLES, drop_soil_to_c_tray_test),
        "D": (D_TRAY_SOIL_DROP_CYCLES, drop_soil_to_d_tray_test),
    }
    cycles, drop_fn = drop_plans[tray_name]

    for cycle_index, _cycle in enumerate(cycles, start=1):
        node.get_logger().info(f"Soil add cycle {cycle_index}/{len(cycles)} tray={tray_name}")
        fill_shovel_from_supply(node, apis)
        drop_fn(node, apis, cycle_index - 1)

    return_shovel(node, apis, set_digital_outputs)


def run_soil_keep_action(node, tray_name: str, result: dict) -> None:
    """Log that measured soil needs no physical corrective movement."""
    node.get_logger().info(
        f"Tray {tray_name} soil state is OK. No action needed. avg_z={result['avg_z']}"
    )


def run_soil_remove_action(node, apis, set_digital_outputs, tray_name: str, result: dict) -> None:
    """Pick shovel, scoop high corners into waste, then return the shovel."""
    pickup_shovel(node, apis, set_digital_outputs)

    if tray_name == "C":
        run_c_tray_custom_remove_action(node, apis)
        return_shovel(node, apis, set_digital_outputs)
        return

    targets = select_target_corners(tray_name, result, TRAY_SOIL_REMOVE_TARGET_NAMES)

    for index, target in enumerate(targets, start=1):
        node.get_logger().info(f"Soil remove cycle {index}/2 target={target['name']}")

        approach_pose = build_corner_pose(apis["posx"], tray_name, target["name"])
        _, _, default_z = corner_xyz_map(tray_name)[target["name"]]
        target_z = target["z"] if target["z"] is not None else result.get("avg_z", default_z)
        scoop_z = target_z + TRAY_SOIL_REMOVE_SCOOP_Z_OFFSET
        scoop_pose = build_corner_pose(apis["posx"], tray_name, target["name"], target_z=scoop_z)
        scoop_lift_pose = build_corner_pose(
            apis["posx"],
            tray_name,
            target["name"],
            target_z=scoop_z + TRAY_SOIL_REMOVE_LIFT_Z_OFFSET,
        )

        move_l(node, apis["movel"], apis["posx"], apis["wait"], f"{target['name']}_APPROACH", normalize_pose(approach_pose))
        move_l(node, apis["movel"], apis["posx"], apis["wait"], f"{target['name']}_SCOOP_DOWN", normalize_pose(scoop_pose))
        shake_scoop_in_tray(node, apis)
        move_l(
            node,
            apis["movel"],
            apis["posx"],
            apis["wait"],
            f"{target['name']}_SCOOP_LIFT",
            normalize_pose(scoop_lift_pose),
        )
        move_l(
            node,
            apis["movel"],
            apis["posx"],
            apis["wait"],
            f"{target['name']}_APPROACH_RETURN",
            normalize_pose(approach_pose),
        )
        dump_to_waste_box(node, apis)

    return_shovel(node, apis, set_digital_outputs)


def run_rock_remove_action(node, apis, set_digital_outputs, tray_name: str, result: dict) -> None:
    """Remove a rock through the fixed, taught pick-and-discard route."""
    del tray_name, result
    node.get_logger().info("Run fixed taught rock removal route")

    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "ROCK_REMOVE_READY",
        ROCK_REMOVE_READY_JOINT,
        check_motion=apis["check_motion"],
    )
    gripper_command(
        node,
        set_digital_outputs,
        apis["wait"],
        "ROCK_PICK_PREPARE_OPEN_0100",
        GRIPPER_HOME_PREPARE,
    )
    move_l(node, apis["movel"], apis["posx"], apis["wait"], "ROCK_PICK", ROCK_REMOVE_PICK_POSE)
    gripper_command(
        node,
        set_digital_outputs,
        apis["wait"],
        "ROCK_GRIP_CLOSE_0010",
        GRIPPER_PICK_CLOSE,
    )
    move_l(node, apis["movel"], apis["posx"], apis["wait"], "ROCK_LIFT", ROCK_REMOVE_LIFT_POSE)
    move_l(node, apis["movel"], apis["posx"], apis["wait"], "ROCK_WASTE_TOP", ROCK_REMOVE_WASTE_TOP_POSE)
    move_l(node, apis["movel"], apis["posx"], apis["wait"], "ROCK_WASTE_DROP", ROCK_REMOVE_WASTE_DROP_POSE)
    gripper_command(
        node,
        set_digital_outputs,
        apis["wait"],
        "ROCK_RELEASE_OPEN_0100",
        GRIPPER_HOME_PREPARE,
    )
    move_l(node, apis["movel"], apis["posx"], apis["wait"], "ROCK_WASTE_TOP_RETURN", ROCK_REMOVE_WASTE_TOP_POSE)
    prepare_home_joint(node, set_digital_outputs, apis["wait"])
    move_j(
        node,
        apis["movej"],
        apis["posj"],
        apis["wait"],
        "HOME_RETURN",
        HOME_JOINT,
        check_motion=apis["check_motion"],
    )
