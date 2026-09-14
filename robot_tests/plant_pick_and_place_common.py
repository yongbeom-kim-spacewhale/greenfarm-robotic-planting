"""Shared constants and sequence for plant pick-and-place nodes."""

from __future__ import annotations

from robot_tests.motion_primitives import (
    GRIPPER_HOME_PREPARE,
    GRIPPER_PICK_CLOSE,
    HOME_JOINT,
    go_home,
    gripper_command,
    move_j,
    move_l,
    prepare_home_joint,
)
from robot_tests.robot_motion_data import (
    PLANT_APPROACH_ACC as APPROACH_ACC,
    PLANT_APPROACH_VEL as APPROACH_VEL,
    PLANT_GRIPPER_WAYPOINT1_PREPARE as GRIPPER_WAYPOINT1_PREPARE,
    PLANT_TCP_NAME as TCP_NAME,
    PLANT_TOOL_NAME as TOOL_NAME,
    PLANT_TRANSFER_ACC as TRANSFER_ACC,
    PLANT_TRANSFER_VEL as TRANSFER_VEL,
    PLANT_WAYPOINT1_POSE as WAYPOINT1_POSE,
    TARGET_TRAY_PLACE_JOINTS,
    TRAY_ENTRY_JOINTS,
    TRAY_PLANT_JOINTS,
)


def run_plant_pick_and_place_sequence(
    node,
    movej,
    movel,
    posj,
    posx,
    set_digital_outputs,
    wait,
    get_current_posx,
    dr_base,
    *,
    source_tray: str,
    plant_index: int,
    target_tray: str,
) -> None:
    """Execute the taught source-pick and selected tray placement route.

    Shared by dashboard and standalone entry nodes: approach, grip, lift,
    traverse to the selected tray, release the plant, and retreat safely.
    """
    tray_entry_joint = TRAY_ENTRY_JOINTS[source_tray]
    plant_path = TRAY_PLANT_JOINTS[source_tray][plant_index]
    place_path = TARGET_TRAY_PLACE_JOINTS[target_tray]

    node.get_logger().info(
        f"Run sequence source={source_tray}, plant={plant_index}, target={target_tray}"
    )

    node.get_logger().info("Step 1/11: go home")
    prepare_home_joint(node, set_digital_outputs, wait)
    move_j(node, movej, posj, wait, "HOME", HOME_JOINT)

    node.get_logger().info("Step 2/11: move to waypoint 1")
    move_l(node, movel, posx, wait, "WAYPOINT1", WAYPOINT1_POSE)

    node.get_logger().info("Step 3/11: wide open gripper at waypoint 1")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "WAYPOINT1_OPEN_1000",
        GRIPPER_WAYPOINT1_PREPARE,
    )

    node.get_logger().info("Step 4/11: move to tray entry joint")
    move_j(
        node,
        movej,
        posj,
        wait,
        f"{source_tray}_entry",
        tray_entry_joint,
        vel=APPROACH_VEL,
        acc=APPROACH_ACC,
    )

    node.get_logger().info("Step 5/11: move to plant pre_pick")
    move_j(
        node,
        movej,
        posj,
        wait,
        f"{source_tray}_plant{plant_index}_pre_pick",
        plant_path["pre_pick"],
        vel=APPROACH_VEL,
        acc=APPROACH_ACC,
    )

    node.get_logger().info("Step 6/11: move to plant pick")
    move_j(
        node,
        movej,
        posj,
        wait,
        f"{source_tray}_plant{plant_index}_pick",
        plant_path["pick"],
        vel=APPROACH_VEL,
        acc=APPROACH_ACC,
    )

    node.get_logger().info("Step 7/11: close gripper strongly at plant pick")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "PICK_CLOSE_0010",
        GRIPPER_PICK_CLOSE,
    )

    node.get_logger().info("Step 8/11: move to plant lift")
    move_j(
        node,
        movej,
        posj,
        wait,
        f"{source_tray}_plant{plant_index}_lift",
        plant_path["lift"],
        vel=TRANSFER_VEL,
        acc=TRANSFER_ACC,
    )

    node.get_logger().info("Step 9/11: move to place top")
    move_j(
        node,
        movej,
        posj,
        wait,
        f"{target_tray}_place_top",
        place_path["top"],
        vel=TRANSFER_VEL,
        acc=TRANSFER_ACC,
    )

    node.get_logger().info("Step 10/11: move to place down")
    move_j(
        node,
        movej,
        posj,
        wait,
        f"{target_tray}_place_down",
        place_path["down"],
        vel=TRANSFER_VEL,
        acc=TRANSFER_ACC,
    )

    node.get_logger().info("Step 11/11: open gripper to release plant and return home")
    gripper_command(
        node,
        set_digital_outputs,
        wait,
        "WIDE_OPEN_0100",
        GRIPPER_HOME_PREPARE,
    )

    go_home(
        node=node,
        set_digital_outputs=set_digital_outputs,
        wait=wait,
        get_current_posx=get_current_posx,
        movel=movel,
        movej=movej,
        posx=posx,
        posj=posj,
        dr_base=dr_base,
    )
