"""arm2link: forward and inverse kinematics for a 2-link planar robot arm."""

from .kinematics import TwoLinkArm, UnreachableError
from .paths import (circle_path, joint_steps_deg, line_path, solve_path,
                    tracking_error)
from .verify import verify_ik
from .workspace import sample_workspace, workspace_summary

__all__ = [
    "TwoLinkArm",
    "UnreachableError",
    "circle_path",
    "line_path",
    "solve_path",
    "tracking_error",
    "joint_steps_deg",
    "verify_ik",
    "sample_workspace",
    "workspace_summary",
]
