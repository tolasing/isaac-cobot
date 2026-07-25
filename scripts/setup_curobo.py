"""Builds a warmed-up cuRobo MotionGen for the CR5. FIRST DRAFT / UNVERIFIED: not run against a
live install. See configs/curobo/cr5.yml's KNOWN GAP note about the URDF's degenerate
velocity/effort limits before expecting motion generation to actually succeed."""

from __future__ import annotations

from pathlib import Path

from curobo.types.base import TensorDeviceType
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

CR5_CUROBO_CONFIG = Path(__file__).resolve().parent.parent / "configs" / "curobo" / "cr5.yml"


def build_motion_gen(config_path: Path = CR5_CUROBO_CONFIG) -> MotionGen:
    tensor_args = TensorDeviceType()
    motion_gen_config = MotionGenConfig.load_from_robot_config(str(config_path), tensor_args=tensor_args)
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen


if __name__ == "__main__":
    build_motion_gen()
    print("[setup_curobo] MotionGen built and warmed up.")
