# neck_fk_01

FK neck component for mGear Shifter. It is the **FK-primary counterpart of
`neck_ik_01`**: the same guide, the same head setup, but the dependency
direction is reversed so that **the FK controllers drive the IK controller**
instead of the FK controls being a read-out of an IK spline solve.

---

## What it does

| Object | Role in `neck_ik_01` | Role in `neck_fk_01` |
| --- | --- | --- |
| `fk*_ctl` chain | Driven read-out of the IK curve solve (non-master transforms) | **Master.** Real parented FK hierarchy, the only rig input |
| `ik_ctl` | Master. Drives master curve → curve-slide → slave curve → path-constrained `div_cns` | **Locked, non-keyable read-out.** Matrix-driven by the last FK control, sits at the neck tip and follows the chain |
| `head_ctl` | Parented under the FK tip, head space switch | Identical |
| master / slave curves, `div_cns`, path constraints, `gear_curveslide2_op`, `gear_intmatrix_op` roll | Core of the IK solve | **Removed.** Replaced by a single one-way matrix connection |

The FK chain is a plain rotate-down hierarchy:
`root → fk0_npo → fk0_ctl → fk1_npo → fk1_ctl → … → fkN_ctl → head_cns → head_ctl`.

Deform joints are placed one per FK control, plus a head joint, matching the
joint layout of `neck_ik_01` (`root` → index 0, `head`/`neck`/`eff` → last).

## Avoiding cyclic redundancy

`neck_ik_01` turns the IK control into the master through an IK spline:

- a **master curve** whose CVs are constrained to the IK / tangent controls,
- a **`gear_curveslide2_op`** producing a **slave curve**,
- **`pathCns`** sliding each `div_cns` along the slave curve,
- a **`gear_intmatrix_op`** roll that reads `ik_ctl.worldMatrix`,
- matrix decomposition of `div_cns` back into `fk_npo.t / .r`.

`neck_fk_01` deletes every one of those operators. The **only** link between
the FK chain and the IK control is:

```
fk_ctl[-1].worldMatrix ──▶ gear_mulmatrix_op ──▶ decomposeMatrix ──▶ ik_cns.t / .r / .s
```

It is strictly one-directional (FK → IK read-out). Because `ik_ctl` has every
transform channel **locked and non-keyable**, nothing — animator, parent,
constraint or reference array — can feed a transform from `ik_ctl` back into
the FK chain. A dependency cycle is therefore impossible **by construction**,
not merely avoided by careful ordering.

### Stretchy-FK without a cycle

The optional squash & stretch measures the live chain length from the
`fk_ctl` world positions (`distanceBetween` per segment) and feeds a
`mgear_squashStretch2` node per division. The trap here is obvious: if the
squash output scaled anything *above* an `fk_ctl`, it would move that control,
change the measured length, and re-trigger the squash — a cycle (this is
exactly what the first draft of this component hit).

It is avoided by writing the squash **only to a leaf `scl_ref` transform**
parented under each `fk_ctl` with nothing below it. Scaling `scl_ref` moves no
control, so the measurement it depends on is never disturbed. The deform joint
rides `scl_ref` (position/orientation from `fk_ctl`, scale from the squash).
The FK control nulls (`fk_npo`) are left completely out of the squash.

## Settings

| Setting | Description |
| --- | --- |
| **Divisions** | Number of FK controls / deform joints along the neck (min 3). |
| **Tangent Controls** | Adds two cosmetic tangent controls, used only by the display curve. |
| **IK Ctl World Ori** | Aligns the locked IK read-out control to world space. |
| **Squash and Stretch Profile** | FCurve profile for the optional volume preservation (`Volume` anim attribute, default 0 = off). |
| **FK Base Reference Array** | Space switch for the base of the neck. Drives `fk0_npo`, which sits above every control, so the whole neck inherits the space without ever touching the IK read-out. |
| **Head Reference Array** | Space switch for the head control (same behaviour as `neck_ik_01`). |

## Animation attributes

- `Volume` — blend for the optional per-segment squash & stretch (0 = no volume preservation).
- `Head Ref` — head control space switch (present when the Head Reference Array has more than one entry).
- `FK Ref` — neck base space switch (present when the FK Base Reference Array has more than one entry).

## Guide

Guide locators are identical to `neck_ik_01` (`root`, `tan0`, `tan1`, `neck`,
`head`, `eff` + `blade`), so an existing `neck_ik_01` guide can simply be
retyped to `neck_fk_01`.

## Removed compared to `neck_ik_01`

- IK stretch: `Max Stretch`, `Max Squash`, `Softness` attributes and the
  `gear_curveslide2_op` (there is no IK spline to stretch).
- `Chicken style IK` and the `IK Reference Array` (there is no animatable IK
  control to reference — use the FK Base Reference Array instead).
- `Lock Ori`, `Tangent 0` / `Tangent 1` scalar attributes (no path solve to
  blend against).
