# ribbon_01

A **ribbon / surface rig** for mGear Shifter — the classic follicle-style
method (antCGI *"Rigging In Maya – Part 18 – Ribbons"*), using
`pointOnSurfaceInfo` + `fourByFourMatrix` (the mGear "rivet" pattern) for the
surface attach.

A NURBS surface is skinned to a real FK-driven chain of **bind joints**, and
any number of **deform joints** ride the surface. Ideal for tails, tentacles,
ropes, straps, antennae, cables, cartoon limbs, lip / brow strips.

## Settings

| Setting | Meaning |
| --- | --- |
| **Mode** | `FK` — bind joints follow a plain FK chain. `IK` — bind joints follow an IK-spline solve (base / mid / tip controls + a curve). `FK/IK` — both are built and the `Fk/Ik Blend` attribute blends between them, with control visibility swapping. |
| **Fk/Ik Blend** | Starting blend value (0 = FK, 1 = IK). |
| **FK Controllers** (`fkNb`) | Number of FK controls / bind joints / IK spline samples (min 2). |
| **Deform Joints** (`jntNb`) | Number of pin transforms / deform joints attached to the surface (min 2), spread evenly along its length. |
| **Tweak Controls** | Adds a tweak control on every pin, between the pin and its deform joint. |
| **Base Reference Array** | Space switch for the base of the ribbon (`ik_cns`). |

## FK / IK matching

`ribbon_01` is a spline component, so mGear's built-in 2-joint `ikFkMatch`
does not apply. Two module functions handle it:

```python
from ribbon_01 import ribbon_IKToFK, ribbon_FKToIK
ribbon_FKToIK(fk_control_names)                       # snap FK to the IK result
ribbon_IKToFK(fk_control_names, ik_control_names)     # snap the IK controls to the FK pose
```

`ribbon_FKToIK` reads the `*_mth` match transforms wired on each FK control
(they track the IK `div_cns`). Wire these to a synoptic / picker button.

## Animation attributes (on the UI host)

| Attribute | Effect |
| --- | --- |
| **Fk/Ik Blend** | (FK/IK mode) 0 = FK, 1 = IK. Blends the bind joints and swaps control visibility. |
| **Twist Start** / **Twist End** | Roll at the base / tip, blended linearly along the length and applied to the bind joints — the skin twists and the pins inherit it. |
| **Roll** | Uniform roll of the whole ribbon. |
| **Volume** | Squash & stretch amount (0 = off, 1 = full volume preservation on the deform joints' cross-section). |
| **Max Stretch** / **Max Squash** | Clamp range for the length ratio that drives the volume factor. |
| **IK Position** / **IK Max Stretch** / **IK Softness** | (IK / FK-IK mode) the curve-slide operator's parameters. |
| **Surface Vis** | Show / hide the NURBS surface (off by default). |
| **Tweak Vis** | Show / hide the tweak controls. |
| **Base Ref** | Base space switch. |

## Hierarchy

```
root
└─ ik_cns
   ├─ fk0_npo → fk0_ctl → … → fk(N-1)_ctl        (FK chain, if Mode != IK)
   ├─ ik0_npo → ik0_ctl   (base)                 (IK spline controls, if Mode != FK)
   │  ikMid_npo → ikMid_ctl   (auto-follows base/tip)
   │  ikTip_npo → ikTip_ctl   (tip)
   │  (tan0_loc under ik0_ctl, tan1_loc under ikTip_ctl)
   └─ blnd0_npo → roll0 → bind0_jnt              (blend layer, one per segment)
      blnd1_npo → roll1 → bind1_jnt
      …

root
├─ mst_crv / slv_crv                             (IK spline curves)
├─ 0_cns → 1_cns → …                             (div_cns chain, path-constrained
│                                                  to slv_crv, inheritsTransform off)
└─ ribbon_root                                   (oriented on the guide)
   ├─ ribbon_srf                                 (NURBS plane, inheritsTransform off,
   │                                              deformed only by the skinCluster)
   └─ pins
      ├─ pin0 (t/r ← surface matrix) → tweak0_npo → tweak0_ctl → attach0 → joint 0
      └─ …
```

* **FK chain** — a plain parented chain, `fk{i}_npo` under `fk{i-1}_ctl`.
* **IK spline** — 3 controls (base / mid / tip). `mst_crv` runs through
  `[ik0_ctl, tan0_loc, tan1_loc, ikTip_ctl]`; `gear_curveslide2_op`
  produces `slv_crv`; each `div_cns` is `pathCns`-constrained along it with
  a twist reference (`gear_intmatrix_op` roll).
* **Blend layer** — `blnd{i}_npo` (flat under `ik_cns`, `inheritsTransform`
  off) is `parentConstrained` (maintainOffset = False, like `chain_01`) to
  the FK control and/or the IK `div_cns`; in FK/IK mode the weights are
  driven by `Fk/Ik Blend` and its reverse. A `roll` transform under it
  carries the twist; the hidden `bind_jnt` (skin influence) hangs off that.
* **Surface** — a `nurbsPlane` (U = length, one span per FK segment; V =
  width), rebuilt to degree 3 U / degree 1 V, 0–1 range, parented onto the
  oriented `ribbon_root` with local zeroed and `inheritsTransform` off,
  deformed **only** by the skinCluster.
* **Pins** — one per deform joint at `U = i / (jntNb-1)`, `V = 0.5`. A
  `pointOnSurfaceInfo` reads the deformed surface; its position + normalized
  U-tangent / normal / V-tangent are packed into a `fourByFourMatrix`
  (U-tangent → X, normal → Y, V-tangent → Z), brought into the pin's parent
  space with `gear_mulmatrix_op`, decomposed, and connected to the pin's
  translate / rotate.
* **Squash & stretch** — the distance between the first and last bind joints
  is normalised by the global rig scale, clamped between **Max Squash** and
  **Max Stretch**, turned into a `1 / sqrt(ratio)` cross-section factor,
  blended towards 1 by **Volume**, and applied to the deform joints' Y and Z
  scale.

## Why there is no cycle

```
fk_ctl ──(skinCluster)──▶ ribbon_srf ──(pointOnSurfaceInfo)──▶ pin ──▶ attach ──▶ joint
```

strictly one direction. The surface is deformed only by the skinCluster from
the bind-joint world positions; nothing downstream of the surface is read
back by the FK chain or the skin. The squash driver is the FK-driven
bind-joint distance, not the deformed surface. `attach` transforms are
leaves.

## Guide

`root` + any number of `#_loc` locators along the ribbon spine, plus a
`blade` for the surface up / normal. Locators shape the guide; the FK and
joint counts are set independently in the component settings.
