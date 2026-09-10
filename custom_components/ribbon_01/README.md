# ribbon_01

A **ribbon / surface rig** component for mGear Shifter.

A NURBS surface is skinned to a chain of FK controllers, and any number of
deform joints ride that surface. A small number of FK controls therefore
drives a smooth, dense joint chain — the classic setup for tails, tentacles,
ropes, straps, antennae, cables, cartoon limbs, lip/brow strips, etc.

The two counts are **independent** and both live in the component settings:

| Setting | Meaning |
| --- | --- |
| **FK Controllers** | Number of animation controls driving the ribbon (min 2). Each one is skinned to one region of the surface. |
| **Deform Joints** | Number of joints attached to the surface (min 2). Spread evenly along its length. |

## How it is built

```
root
└─ ik_cns                         (base space-switch null)
   └─ fk0_npo → fk0_ctl
      └─ fk0_ref                  (skinned to the surface)
      └─ fk1_npo → fk1_ctl
         └─ fk1_ref
         └─ …

root
└─ ribbon_root                    (oriented: local +X = chain direction)
   ├─ ribbon_srf                  (NURBS plane, inheritsTransform = False,
   │                               skinned to fk*_ref, hidden by default)
   ├─ attach0_npo → attach0       → deform joint 0
   ├─ attach1_npo → attach1       → deform joint 1
   └─ …
```

1. **Guide sampling** — the guide is a simple `root` + `#_loc` multi chain
   plus a `blade`. A temporary degree-3 curve is fitted through the locators
   and sampled by **uniform arc length** to place the FK controls.
2. **FK chain** — a real parented FK hierarchy. Under each control sits an
   `fk*_ref` transform; the surface is skinned to these.
3. **Surface** — a `nurbsPlane`, degree 3 along its length (U), linear across
   its width (V), with one U span per FK segment. It is rebuilt to a 0–1
   parameter range, parented under the oriented `ribbon_root`, set
   `inheritsTransform = False`, and skinned to the `fk*_ref` chain
   (`maximumInfluences = 2`, smooth dropoff — refine the weights to taste).
4. **Joint attach** — for each deform joint, a `pointOnSurfaceInfo` reads the
   surface at `parameterU = i / (jntNb - 1)`, `parameterV = 0.5`. Its
   position + normalized U-tangent / normal / V-tangent are packed into a
   `fourByFourMatrix` (U-tangent → X, normal → Y, V-tangent → Z), decomposed,
   and connected to the `attach` transform (the classic "rivet"). The joint
   scale is driven by the global rig scale.

## Why there is no cycle

```
fk_ctl ──(skinCluster)──▶ ribbon_srf ──(pointOnSurfaceInfo)──▶ attach ──▶ joint
```

strictly one direction. The surface has `inheritsTransform = False` and is
deformed **only** by the skinCluster (from the `fk*_ref` world positions);
nothing downstream of the surface is read back by the FK chain or the skin.
The `attach` transforms are leaves.

## Settings

| Setting | Description |
| --- | --- |
| **FK Controllers** | Number of FK controls (min 2). |
| **Deform Joints** | Number of joints attached to the surface (min 2). |
| **Base Reference Array** | Space switch for the base of the ribbon (drives `ik_cns`, above the whole FK chain). |

## Animation attributes

- `Surface Vis` — show / hide the underlying NURBS surface (off by default).
- `Base Ref` — base space switch (present only when the Base Reference Array
  has more than one entry).

## Guide

`root` + any number of `#_loc` locators along the ribbon spine, plus a
`blade` giving the surface up / normal direction. Add or remove `#_loc`
locators to change the *shape* of the guide; the number of FK controls and
joints is set independently in the component settings, not by the locator
count.
