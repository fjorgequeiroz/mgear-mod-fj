# ribbon_01

A **ribbon / surface rig** for mGear Shifter — the antCGI *"Rigging In Maya
– Part 18 – Ribbons"* method, with the follicles replaced by a single
**`uvPin`** node (Maya 2020+).

A NURBS surface is skinned to a small chain of FK-driven **bind joints**, and
any number of **deform joints** ride the surface via `uvPin`. Ideal for tails,
tentacles, ropes, straps, antennae, cables, cartoon limbs, lip / brow strips.

## Settings

| Setting | Meaning |
| --- | --- |
| **FK Controllers** (`fkNb`) | Number of FK controls, each driving one bind joint the surface is skinned to (min 2). The middle ones auto-follow their neighbours. |
| **Deform Joints** (`jntNb`) | Number of pin transforms / deform joints attached to the surface (min 2), spread evenly along its length. |
| **Tweak Controls** | Adds a tweak control on every pin, between the pin and its deform joint. |
| **Base Reference Array** | Space switch for the base of the ribbon (`ik_cns`, above the FK chain). |

## Build pipeline

```
root
└─ ik_cns
   └─ fk0_npo → fk0_ctl → bind0_jnt
      └─ fk1_npo → fk1_ctl → bind1_jnt      (middle: parented to ik_cns,
      │                                       parentConstraint auto-follow)
      └─ … → fkN_ctl → bindN_jnt

root
└─ ribbon_root                       (local +X = chain direction)
   ├─ ribbon_srf                     (lofted NURBS surface, skinned to bind*_jnt)
   ├─ ribbon_srfOrig                 (static duplicate, uvPin originalGeometry)
   └─ pins
      ├─ pin0_npo → pin0  (+ tweak0_ctl) → attach0 → deform joint 0
      ├─ pin1_npo → pin1  (+ tweak1_ctl) → attach1 → deform joint 1
      └─ …
```

1. **Surface** — two rail curves are built, offset ±half-width across each FK
   transform's local Z, and **lofted** (degree 1, uniform). The loft
   guarantees U runs along the length and V across the width — no reliance on
   `nurbsPlane`'s axis convention. It is then `rebuildSurface`d to degree 3 U
   / degree 1 V, one U span per FK segment, `keepRange = 0` (0–1 param range).
2. **Bind** — the surface is skinned to `bind*_jnt` (one per FK control),
   `maximumInfluences = 2`, smooth dropoff. Refine the weights to taste.
3. **`uvPin`** — one node, fed the live skinned surface (`deformedGeometry`),
   a static duplicate (`originalGeometry`), and `jntNb` coordinates at
   `U = i / (jntNb-1)`, `V = 0.5`. Axes are set explicitly: **X = U tangent
   (length), Y = surface normal, Z = width**. `relativeSpaceMode = 1` outputs
   matrices relative to the `pins` group; each `outputMatrix[i]` feeds a pin's
   `offsetParentMatrix`.
4. **Squash & stretch** — the distance between the first and last bind joints
   (FK-driven, so no cycle) drives a `sqrt(rest / current)` factor, blended by
   the `Volume` attribute, applied to the deform joints' Y and Z scale (X = the
   length axis keeps only the global rig scale).
5. **Twist** — free: rotating the tip control around X twists the surface, the
   pins follow.

## Why there is no cycle

```
fk_ctl ──(skinCluster)──▶ ribbon_srf ──(uvPin)──▶ pin ──▶ attach ──▶ joint
```

strictly one direction. The surface is deformed only by the skinCluster from
the bind-joint world positions; nothing downstream of the surface is read back
by the FK chain or the skin. The squash driver is the FK-driven bind-joint
distance, not the deformed surface. `attach` transforms are leaves.

## Animation attributes

- `Surface Vis` — show / hide the NURBS surface (off by default).
- `Volume` — squash & stretch amount (0 = off, 1 = full).
- `Mid Follow` — how strongly the middle FK controls follow their neighbours
  (present when `fkNb > 2`).
- `Tweak Vis` — show / hide the tweak controls (present when Tweak Controls is
  on).
- `Base Ref` — base space switch (present when the Base Reference Array has
  more than one entry).

## Guide

`root` + any number of `#_loc` locators along the ribbon spine, plus a `blade`
for the surface up / normal. Locators shape the guide; the FK and joint counts
are set independently in the component settings.

## Requirements

`uvPin` requires **Maya 2020 or newer**.
