---
description: Audit a USD asset (or a scene that references it) for duplicate/ghost PhysX collider schemas — the pattern found repeatedly on the CR5 gripper, mug, and scanner-assembly parts. Use whenever an object passes through things it shouldn't, refuses to close/grasp cleanly, or before trusting a part's collision setup.
argument-hint: [usd-file-path-or-composed-scene-prim-path]
---

# Audit colliders for ghost/duplicate PhysX schemas

This repo has hit the same collision-authoring bug in at least four
different places (CR5 gripper links, `/World/mug`, `finger_print_scanner`,
`main_holder`) — always some variant of "there's more than one collider
opinion stacked on a prim, and only one of them is doing anything." Don't
re-derive this from scratch; run the checks below.

## The three failure modes, in order of how often they show up

1. **Stale approximation schema stacked on the active one.** A prim can
   have `PhysxSDFMeshCollisionAPI`, `PhysxConvexHullCollisionAPI`,
   `PhysxConvexDecompositionCollisionAPI`, and/or
   `PhysxTriangleMeshSimplificationCollisionAPI` *all* applied
   simultaneously — only the one matching `MeshCollisionAPI`'s
   `approximation` attribute is actually active; the rest are cosmetic
   cruft left over from cycling through the GUI's Approximation dropdown.
   Harmless numerically, but hides real asymmetries (e.g. one finger of a
   gripper had an extra schema the other didn't) and is worth cleaning for
   consistency.
2. **`CollisionAPI`/`MeshCollisionAPI` on a prim with no real geometry.**
   If applied to a parent Xform whose actual mesh lives in a *sibling*
   prim (not a descendant), it's silently inert — no error, it just does
   nothing. This is different from a **native USD instance** (`prim.
   IsInstance()==True`, referencing a `Sdf.SpecifierClass` "Prototypes"
   scope): that pattern IS legitimate — the real geometry is reachable via
   `prim.GetPrototype()`, but `GetAllChildren()`/default traversal won't
   show it. Don't flag instances as inert without checking the prototype.
3. **The GUI edit landed in the wrong file.** Editing a *referenced*
   asset's collider live in the GUI (Colliders Preset, changing
   Approximation, Delete) authors the change as an override in whichever
   top-level file is currently open — not in the referenced source asset.
   A part's own `.usd` can look perfectly clean standalone while the
   composed scene carries an independent, possibly-conflicting collider.
   **Always audit the fully composed scene the part is actually used in,
   not just the source file** — check `prim.GetPrimStack()` to see which
   layer(s) actually author the `apiSchemas` you're looking at.

## How to check (headless, no GUI needed)

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True}, experience="")
from pxr import Usd, UsdPhysics, PhysxSchema

APPROX_SCHEMAS = [
    ("sdf", PhysxSchema.PhysxSDFMeshCollisionAPI),
    ("convexDecomposition", PhysxSchema.PhysxConvexDecompositionCollisionAPI),
    ("convexHull", PhysxSchema.PhysxConvexHullCollisionAPI),
    ("meshSimplification", PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI),
]

stage = Usd.Stage.Open(PATH)  # the standalone source file, OR the composed scene
for _ in range(60):
    simulation_app.update()  # let async import/reference resolution settle

# Usd.PrimRange.AllPrims (not the default predicate!) — USD silently skips
# instancing Prototypes scopes under the default traversal predicate.
for prim in Usd.PrimRange.AllPrims(stage.GetPseudoRoot()):
    has_c = prim.HasAPI(UsdPhysics.CollisionAPI)
    has_mc = prim.HasAPI(UsdPhysics.MeshCollisionAPI)
    if not (has_c or has_mc):
        continue
    approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() if has_mc else None
    active = [name for name, schema in APPROX_SCHEMAS if prim.HasAPI(schema)]
    stack = [s.layer.identifier for s in prim.GetPrimStack()]
    print(prim.GetPath(), "approx=", approx, "applied=", active, "layers=", stack)
    if len(active) > 1:
        print("  !! stale schema(s):", [n for n in active if n != approx])
simulation_app.close()
```

Run it via `/isaac-sim/python.sh <script>.py` (this repo has no working
`pytest`/`pip` — see the `headless-test` skill). If auditing a part that's
referenced into a larger scene (e.g. `assets/mefron/factory floor/mefron.usd`,
or a Grasp Editor scene), run it **both** against the standalone source
file **and** against the composed scene at the part's actual prim path —
they can disagree (see failure mode 3).

## Fixing what you find

- Stale extra schema: `prim.RemoveAPI(PhysxSchema.PhysxConvexHullCollisionAPI)`
  (or whichever), keeping the one matching `approximation`. Save with
  `stage.GetRootLayer().Save()`.
- If the schema is authored independently in *both* the source file and a
  composed scene's override (check `GetPrimStack()` — if the composed
  scene's own layer shows up with its own `apiSchemas` value, not just a
  reference arc, it's a real independent copy), fix both, don't assume
  fixing the source file cascades to the override.
- If a part's *only* collider turns out to live entirely as a composed-
  scene override (source file has none at all), that's not necessarily a
  bug — just make sure you're editing the right layer, not creating a
  second copy.

After fixing, re-run the audit script to confirm exactly one active
approximation schema per collider prim.
