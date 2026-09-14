"""Shared compliance/force-control helpers."""

from __future__ import annotations

from robot_tests.safety_state import ensure_motion_allowed


def extract_fz(force: object) -> float:
    """Return base/tool Z force from the six-axis force vector."""
    if not isinstance(force, (list, tuple)) or len(force) < 3:
        raise TypeError(f"Unsupported force type: {type(force).__name__}")
    return float(force[2])


def safe_release_compliance(node, release_compliance_ctrl) -> None:
    """Best-effort force-control release used in exception/finally cleanup paths."""
    try:
        release_compliance_ctrl()
        node.get_logger().info("release_compliance_ctrl done")
    except Exception as exc:
        node.get_logger().warn(f"release_compliance_ctrl failed: {exc}")


def safe_return_to_start(node, movel, posx, wait, start_pose: list[float] | None, *, vel: float, acc: float) -> None:
    """Recover the TCP to its pre-probe pose without hiding recovery failures."""
    if start_pose is None:
        node.get_logger().warn("start_pose not available. Skip return.")
        return

    try:
        node.get_logger().info(f"Return to start_pose z={start_pose[2]:.3f}")
        ensure_motion_allowed()
        movel(posx(start_pose), vel=vel, acc=acc)
        wait(1.0)
    except Exception as exc:
        node.get_logger().error(f"Failed to return to start_pose: {exc}")
