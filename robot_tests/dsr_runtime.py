# Doosan 로봇 파이썬 모듈 경로를 잡아주는 런타임 보조 코드다.
"""Runtime helpers for loading Doosan robot Python modules."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _find_repo_root(start: Path) -> Path | None:
    """Find the workspace root that contains the sibling ws_dsr workspace."""
    for candidate in (start, *start.parents):
        if (candidate / "ws_dsr").is_dir():
            return candidate
    return None

def bootstrap_dsr_python() -> None:
    """Make Doosan SDK modules importable when the ROS workspace is not overlaid.

    Robot nodes call this before importing ``DR_init``. It changes Python's
    module search path only and never connects to or moves the robot.
    """
    if importlib.util.find_spec("DR_init") is not None:
        return

    current_file = Path(__file__).resolve()
    repo_root = _find_repo_root(current_file)
    if repo_root is None:
        return

    py_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        repo_root / "ws_dsr/install/dsr_common2/lib" / py_version / "site-packages",
        repo_root / "ws_dsr/install/dsr_common2/lib/dsr_common2/imp",
        repo_root / "ws_dsr/src/cobot_rg2/doosan-robot2/dsr_common2/imp",
    ]

    for candidate in candidates:
        candidate_str = str(candidate)
        if candidate.is_dir() and candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)
