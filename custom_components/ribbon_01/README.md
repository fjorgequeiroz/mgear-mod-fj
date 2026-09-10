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
| **FK Controllers** (`fkNb`) | Number of FK controls, each driving one bind joint the surface is skinned to (min 2). |
| **Deform Joints** (`jntNb`) | Number of pin transforms / deform joints attached to the surface (min 2), spread evenly along its length. |
| **Tweak Controls** | Adds a tweak control on every pin, between the pin and its deform joint. |
| **Base Reference Array** | Space switch for the base of the ribbon (`ik_cns`, above the FK chain). |

## Animation attributes (on the UI host)

| Attribute | Effect |
| --- | --- |
| **Twist Start** / **Twist End** | Roll at the base / tip, blended linearly along the length and applied to the bind joints — the skin twists and the pins inherit it. |
| **Roll** | Uniform roll of the whole ribbon (added to every bind joint's length-axis rotation). |
| **Volume** | Squash & stretch amount (0 = off, 1 = full volume preservation on the deform joints' cross-section). |
| **Max Stretch** / **Max Squash** | Clamp range for the length ratio that drives the volume factor. |
| **Surface Vis** | Show / hide the NURBS surface (off by default). |
| **Tweak Vis** | Show / hide the tweak controls (present when Tweak Controls is on). |
| **Base Ref** | Base space switch (present when the Base Reference Array has more than one entry). |

## Hierarchy

```
root
└─ ik_cns
   └─ fk0_npo → fk0_ctl → roll0 → bind0_jnt      ← PLAIN parented FK chain
      └─ fk1_npo → fk1_ctl → roll1 → bind1_jnt
         └─ … → fk(N-1)_ctl → roll(N-1) → bind(N-1)_jnt

root
└─ ribbon_root                       (oriented on the guide)
   ├─ ribbon_srf                     (NURBS plane, local zeroed, skinned to bind*_jnt)
   ├─ ribbon_srfShapeOrig            (intermediate from the skinCluster)
   └─ pins
      ├─ pin0 (t/r ← surface matrix) → tweak0_npo → tweak0_ctl → attach0 → joint 0
      └─ …
```

* **FK chain** — a plain parented chain. `fk{i}_npo` is parented to
  `fk{i-1}_ctl`; no reparenting, no auto-follow constraints. Under each
  control a `roll` transform receives the twist rotation, and a hidden
  `bind_jnt` (the skin influence) hangs off that.
* **Surface** — a `nurbsPlane` (U = length, one span per FK segment; V =
  width), rebuilt to degree 3 U / degree 1 V, `keepRange = 0` (0–1 param
  range). Parented onto the oriented `ribbon_root` with a zeroed local
  transform, then deformed **only** by the skinCluster.
* **Pins** — one per deform joint at `U = i / (jntNb-1)`, `V = 0.5`. A
  `pointOnSurfaceInfo` reads the deformed surface; its position + normalized
  U-tangent / normal / V-tangent are packed into a `fourByFourMatrix`
  (U-tangent → X, normal → Y, V-tangent → Z), brought into the pin's parent
  space with `gear_mulmatrix_op`, decomposed, and connected to the pin's
  translate / rotate.
* **Squash & stretch** — the distance between the first and last bind joints
  (FK-driven, no cycle) is normalised by the global rig scale, clamped
  between **Max Squash** and **Max Stretch**, turned into a
  `1 / sqrt(ratio)` cross-section factor, blended towards 1 by **Volume**,
  and applied to the deform joints' Y and Z scale (X keeps only the global
  rig scale).

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
