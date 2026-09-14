"""대시보드와 자식 작업 프로세스가 공유하는 소프트웨어 안전정지 플래그."""

from __future__ import annotations

from pathlib import Path


SAFETY_STOP_FLAG_PATH = Path("/tmp/robot_tests_safety_stop")


def activate_safety_stop() -> None:
    """새 이동 명령을 막는 안전정지 플래그를 생성한다."""
    SAFETY_STOP_FLAG_PATH.touch(exist_ok=True)


def clear_safety_stop() -> None:
    """안전복구 후 이동 허용 플래그를 정상 상태로 되돌린다."""
    SAFETY_STOP_FLAG_PATH.unlink(missing_ok=True)


def ensure_motion_allowed() -> None:
    """모든 SDK 이동 직전에 호출하여 정지 상태의 후속 이동을 차단한다."""
    if SAFETY_STOP_FLAG_PATH.exists():
        raise RuntimeError("소프트웨어 안전정지 상태이므로 이동 명령을 실행하지 않음")
