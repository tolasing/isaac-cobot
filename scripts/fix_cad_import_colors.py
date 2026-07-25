"""Corrects a color-space bug in Isaac Sim's CAD Converter: STEP's COLOUR_RGB entities are sRGB,
but the converter writes them verbatim into diffuseColor/emissiveColor, which USD/Hydra treats as
linear -- confirmed by diffing a converted file's values against the source STEP bit-for-bit.
Writes to a new file by default; --in-place to overwrite."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from isaacsim import SimulationApp

if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdShade  # noqa: E402

_COLOR_INPUT_NAMES = ("diffuseColor", "emissiveColor")


def srgb_to_linear(c: float) -> float:
    """Standard sRGB EOTF (IEC 61966-2-1), applied per channel."""
    if c <= 0.04045:
        return c / 12.92
    return ((c + 0.055) / 1.055) ** 2.4


def fix_colors(usd_path: Path) -> list[tuple[str, str, tuple, tuple]]:
    """Converts every diffuseColor/emissiveColor on a UsdPreviewSurface shader from (assumed) sRGB
    to linear, in place on the open stage. Returns (prim_path, input_name, old_value, new_value)
    for everything changed, so callers can print/verify before saving."""
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Could not open {usd_path}")

    changes = []
    for prim in stage.Traverse():
        shader = UsdShade.Shader(prim)
        if not shader:
            continue
        for name in _COLOR_INPUT_NAMES:
            inp = shader.GetInput(name)
            if inp is None:
                continue
            old_val = inp.Get()
            if old_val is None:
                continue
            new_val = type(old_val)(*(srgb_to_linear(c) for c in old_val))
            inp.Set(new_val)
            changes.append((str(prim.GetPath()), name, tuple(old_val), tuple(new_val)))

    stage.GetRootLayer().Save()
    return changes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("usd_path", type=Path, help="CAD-Converter-produced USD file to correct.")
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Overwrite usd_path directly instead of writing a new *_color_fixed.usd file.",
    )
    args, _unknown = parser.parse_known_args()

    if not args.usd_path.is_file():
        print(f"[fix_cad_import_colors] {args.usd_path} not found.", file=sys.stderr, flush=True)
        simulation_app.close()
        sys.exit(1)

    if args.in_place:
        target_path = args.usd_path
    else:
        target_path = args.usd_path.with_name(f"{args.usd_path.stem}_color_fixed{args.usd_path.suffix}")
        shutil.copy2(args.usd_path, target_path)

    changes = fix_colors(target_path)
    print(f"[fix_cad_import_colors] corrected {len(changes)} color input(s) in {target_path}", flush=True)
    for prim_path, name, old_val, new_val in changes:
        print(f"  {prim_path}.{name}: {old_val} -> {new_val}", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
