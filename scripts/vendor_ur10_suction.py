"""One-time (re-runnable) vendoring of just the suction-gripper asset out of Isaac Sim's bundled
UR10 (whose `ur10.usd` has a `Gripper` VariantSet with `Short_Suction`/`Long_Suction` options) into
robots/ur10_suction/, via omni.kit.usd.collect's Collector -- the same "Collect Asset" mechanism
robots/franka_panda/SOURCE.md documents doing manually through the Content Browser, just scripted
instead. Collects grippers/short_gripper.usd directly (NOT the whole ur10.usd), so the ~21MB UR10
arm body/configuration never gets pulled in -- only the gripper end-effector is needed, to attach to
scripts/mefron.py's arm 2. See robots/ur10_suction/SOURCE.md for why this asset was vendored.

Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/vendor_ur10_suction.py --headless
"""

from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

from mefron_lib.kit_bootstrap import preload_real_packaging  # noqa: E402

preload_real_packaging()

# Public, unauthenticated S3 bucket -- same content Isaac Sim's own Content Browser "Isaac Sim"
# bookmark tree browses under Robots/UniversalRobots/ur10/grippers/. Confirmed live (curl) reachable
# from this environment. Short_Suction (not Long_Suction) is the variant we want -- see
# robots/ur10_suction/SOURCE.md.
SOURCE_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
    "/Isaac/Robots/UniversalRobots/ur10/grippers/short_gripper.usd"
)
TARGET_DIR = Path(__file__).resolve().parent.parent / "robots" / "ur10_suction"


async def _collect() -> tuple[bool, str]:
    from omni.kit.usd.collect import Collector

    collector = Collector(usd_path=SOURCE_USD_URL, collect_dir=str(TARGET_DIR) + "/")
    return await collector.collect()


def main() -> None:
    print(f"[vendor_ur10_suction] collecting {SOURCE_USD_URL}", flush=True)
    print(f"[vendor_ur10_suction] into {TARGET_DIR}", flush=True)
    success, collected_root_usd = simulation_app.run_coroutine(_collect())
    if success:
        print(f"[vendor_ur10_suction] PASS: collected root at {collected_root_usd}", flush=True)
    else:
        print("[vendor_ur10_suction] FAIL: collect() reported failure.", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
