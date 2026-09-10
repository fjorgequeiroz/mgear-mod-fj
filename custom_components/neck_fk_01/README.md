# neck_fk_01

FK-primary neck component for mGear Shifter, derived from a verbatim copy of
**`neck_ik_01`**. It keeps `neck_ik_01`'s complete IK-spline solve and the same
deform-joint output, and changes exactly one thing: **who is the master**.

Same idea as `spine_FK_01_horizontal`.

---

## `neck_ik_01` vs `neck_fk_01`

| | `neck_ik_01` | `neck_fk_01` |
| --- | --- | --- |
| Master | `ik_ctl` (free input at the neck tip) | `fk*_ctl` chain (real parented FK hierarchy) |
| FK controls | Fake — their parent null `fk_npo` is matrix-driven by the spline, so they float on the IK result | Real, keyable, parent the whole component |
| IK / tangent controls | Masters of `mst_crv` | Keyable **children of the FK chain** — carried by FK through plain DAG parenting, still animatable for an IK offset on top of FK |
| `mst_crv → gear_curveslide2 → slv_crv → pathCns → div_cns` | ✔ | ✔ (unchanged) |
| Twist refs, `gear_intmatrix_op` roll, `Lock Ori` | ✔ | ✔ (unchanged) |
| Deform joints | `jnt_pos` on `fk_ctl` | `jnt_pos` on the leaf `scl_ref` under each `div_cns` |
| `div_cns → fk_npo` matrix read-out | ✔ (this is what makes IK the master) | **removed** |
| `chickenStyleIK`, `ikrefarray` | ✔ | replaced by **`fkrefarray`** on the neck base |

## Hierarchy

```
root
└─ fk0_npo → fk0_ctl
   └─ fk1_npo → fk1_ctl
      └─ … → fkN_ctl
         ├─ ik_cns → ik_ctl          (+ tan1_loc/ctl under ik_ctl)
         └─ head_cns → head_ctl
   └─ (tan0_loc/ctl under fk0_ctl)

root
└─ 0_cns → 1_cns → … → N_cns         (div_cns chain, inheritsTransform = False)
     └─ N_scl_ref  → deform joint N
```

`ik_cns` is parented under the last FK control, `tan0` under the first. There
is **no connection or constraint** from FK to IK — it is pure hierarchy.

## Why there is no cycle

The whole graph is one-directional:

```
fk_ctl ──(DAG parent)──▶ ik_ctl / tan_ctl
       ──(mst_crv CV cns)──▶ gear_curveslide2 ──▶ slv_crv
       ──(pathCns)──▶ div_cns   (inheritsTransform = False, own chain
                                  under root — NOT under the FK hierarchy)
       ──▶ scl_ref (leaf) ──▶ deform joint
```

* The `div_cns → fk_npo` matrix read-out of `neck_ik_01` — the connection
  that fed the spline result back onto the FK controls and made IK the
  master — is **deleted**. Nothing writes to `fk_npo` any more; it is a
  plain control offset null.
* `div_cns` has `inheritsTransform = False` and lives in its own chain under
  `root`. It only moves along the slave curve via the path constraint, so the
  curve solve that reads the FK/IK controls cannot loop back through the DAG.
* Nothing in the FK chain, and none of the curves, ever read `div_cns`,
  `scl_ref`, or the joints.
* The squash & stretch writes **only** to the leaf `scl_ref` transforms
  (no children), so it moves no control and disturbs no measurement.

## Settings

| Setting | Description |
| --- | --- |
| **Max Stretch** / **Max Squash** / **Softness** | IK-spline curve-slide behaviour (identical to `neck_ik_01`). |
| **Divisions** | Number of FK controls and of deform joints / spline samples (min 3). |
| **Tangent Controls** | Exposes `tan0_ctl` / `tan1_ctl` as real controls instead of hidden locators. |
| **IK Ctl World Ori** | Aligns the IK control to world space. |
| **Squash and Stretch Profile** | FCurve profile for the per-division volume preservation. |
| **FK Base Reference Array** | Space switch for the base of the neck. Drives `fk0_npo` (above every control), so the child IK / tangent controls inherit it too. Replaces `neck_ik_01`'s IK Reference Array. |
| **Head Reference Array** | Space switch for the head control (identical to `neck_ik_01`). |

## Animation attributes

- `Max Stretch`, `Max Squash`, `Softness` — IK-spline stretch behaviour.
- `Lock Ori` — blends the last division's orientation towards the IK control (1 = locked).
- `Tangent 0` / `Tangent 1` — tangent length multipliers.
- `Volume` — squash & stretch blend (0 = off, 1 = full).
- `Head Ref` / `FK Ref` — space switches (present only when their array has more than one entry).

## Guide

Guide locators are identical to `neck_ik_01` (`root`, `tan0`, `tan1`, `neck`,
`head`, `eff` + `blade`), so an existing `neck_ik_01` guide can be retyped to
`neck_fk_01` directly.
