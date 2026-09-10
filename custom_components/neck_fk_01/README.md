# neck_fk_01

FK-primary neck component for mGear Shifter. It is the **FK counterpart of
`neck_ik_01`**: it keeps the *exact same IK-spline solve and the same deform
output*, but the **FK controllers are the master and they drive the IK
control**, instead of the IK control being the master and the FK controls
being a fake read-out.

Built on the same idea as `spine_FK_01_horizontal`.

---

## How it works

### The FK chain is the master

`root → fk0_npo → fk0_ctl → fk1_npo → fk1_ctl → … → fkN_ctl`

A real, parented FK hierarchy. Every `fk*_ctl` is keyable (T/R/S), rotate
order `ZXY`, mirror-inverted. The head control is parented under `fkN_ctl`.

### The IK / tangent controls are children of the FK chain

| Control | Parent | Effect |
| --- | --- | --- |
| `ik_ctl` (`ik_cns`) | `fk_ctl[-1]` | Rides the FK tip through plain DAG parenting. Still keyable, so an animator can add an IK-style offset **on top of** the FK pose. |
| `tan1_ctl` / `tan1_loc` | `ik_ctl` | Follows the FK tip. |
| `tan0_ctl` / `tan0_loc` | `fk_ctl[0]` | Follows the FK base. |

There is **no connection or constraint** from FK to IK — it is pure hierarchy.

### The IK spline still drives the joints

Same as `neck_ik_01`:

```
fk_ctl / ik_ctl / tan_ctl
   └─ mst_crv (CV constrained)
        └─ gear_curveslide2_op ──▶ slv_crv
             └─ pathCns ──▶ div_cns[i]   (inheritsTransform = False)
                  └─ scl_ref[i]  (leaf)
                       └─ deform joint i
```

Plus the twist references (`gear_intmatrix_op` roll from `intMRef` and
`ik_ctl`), the `Lock Ori` blend on the last division, and the per-division
squash & stretch driven by the slave-curve arc length.

The deform joints therefore follow a **smooth interpolated spline** shaped by
the FK pose (and any IK offset), not the raw FK segments.

## Why there is no cycle

The whole graph is one-directional:

```
fk_ctl ──(DAG parent)──▶ ik_ctl / tan_ctl ──(curve)──▶ div_cns ──▶ scl_ref ──▶ joint
```

* `div_cns` has `inheritsTransform = False` and is parented in its own chain
  under `root`. It is **not** under the FK hierarchy; it only moves along the
  slave curve via the path constraint. So the curve solve reading the FK/IK
  controls and writing `div_cns` cannot loop back through the DAG.
* Nothing in the FK chain, and none of the curves, ever read `div_cns`,
  `scl_ref`, or the joints.
* The squash & stretch writes **only** to the leaf `scl_ref` transforms
  (no children), so it moves no control and cannot disturb any measurement.

This is structurally identical to `neck_ik_01`; only the ownership of the IK
and tangent controls moved from "master" to "child of the FK chain".

## Settings

| Setting | Description |
| --- | --- |
| **Max Stretch** / **Max Squash** / **Softness** | IK-spline curve-slide behaviour (same as `neck_ik_01`). |
| **Divisions** | Number of deform joints / spline samples (min 3). Also the number of FK controls. |
| **Tangent Controls** | Exposes `tan0_ctl` / `tan1_ctl` as real controls instead of hidden locators. |
| **IK Ctl World Ori** | Aligns the IK control to world space. |
| **Squash and Stretch Profile** | FCurve profile for the per-division volume preservation. |
| **FK Base Reference Array** | Space switch for the base of the neck (drives `fk0_npo`, above every control, so the child IK / tangent controls inherit it too). |
| **Head Reference Array** | Space switch for the head control (same as `neck_ik_01`). |

## Animation attributes

- `Max Stretch`, `Max Squash`, `Softness` — IK-spline stretch behaviour.
- `Lock Ori` — blends the last division's orientation towards the IK control (1 = locked).
- `Tangent 0` / `Tangent 1` — tangent length multipliers.
- `Volume` — squash & stretch blend (0 = off).
- `Head Ref` / `FK Ref` — space switches (present when their array has more than one entry).

## Guide

Guide locators are identical to `neck_ik_01` (`root`, `tan0`, `tan1`, `neck`,
`head`, `eff` + `blade`), so an existing `neck_ik_01` guide can simply be
retyped to `neck_fk_01`.

## Difference from `neck_ik_01`

| | `neck_ik_01` | `neck_fk_01` |
| --- | --- | --- |
| Master | `ik_ctl` | `fk*_ctl` chain |
| FK controls | Fake (driven read-out of the spline) | Real, keyable, parent everything |
| IK control | Master of the spline | Keyable child of `fk_ctl[-1]`, adds offset on top of FK |
| Joints | IK spline | IK spline (unchanged) |
| `chickenStyleIK`, `ikrefarray` | Yes | Replaced by `fkrefarray` on the neck base |
