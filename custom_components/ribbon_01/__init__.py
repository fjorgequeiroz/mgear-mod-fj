"""Component Ribbon 01 module.

A ribbon / surface rig - the classic follicle-style method (antCGI
"Rigging In Maya - Part 18 - Ribbons"), using ``pointOnSurfaceInfo`` +
``fourByFourMatrix`` (the mGear "rivet" pattern) for the surface attach.

Pipeline
--------
1. A NURBS plane is laid along the guide, rebuilt to degree 3 along the
   length (U) / degree 1 across the width (V), one U span per FK segment,
   0-1 parameter range.
2. A chain of hidden **bind joints** (one per FK segment) is blended between
   an FK chain and an IK-spline solve (see Mode below). The surface is
   skinned to those bind joints.
3. ``jntNb`` **pin transforms** are attached to the surface at evenly
   spaced U values (V = 0.5) via ``pointOnSurfaceInfo`` -> matrix. Each pin
   carries a deform joint, plus an optional tweak control.
4. Extra animation features on the UI host: Twist Start / Twist End, Roll,
   Volume + Max Stretch / Max Squash.

Mode
----
* **FK** - the bind joints follow a plain parented FK chain.
* **IK** - the bind joints follow an IK-spline solve driven by base / mid /
  tip controls and a curve-slide operator.
* **FK/IK** - both are built and a ``Fk/Ik Blend`` attribute blends the bind
  joints between them; control visibility swaps with the blend. Use the
  module functions ``ribbon_IKToFK`` / ``ribbon_FKToIK`` to match one side
  to the other.

No cycle: (fk_ctl | ik_ctl -> spline) -> bind_jnt -> skinCluster -> surface
-> pointOnSurfaceInfo -> pin -> joint, one direction. The volume driver is
the distance between the bind joints, never the deformed surface.
"""

import mgear.pymaya as pm
from mgear.pymaya import datatypes

from mgear.shifter import component

from mgear.core import node, vector, curve, applyop
from mgear.core import attribute, transform, primitive

#############################################
# COMPONENT
#############################################


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""

        self.normal = self.guide.blades["blade"].z * -1
        self.binormal = self.guide.blades["blade"].x

        self.fk_number = max(2, self.settings["fkNb"])
        self.jnt_number = max(2, self.settings["jntNb"])
        self.tweak = self.settings["tweakControls"]

        self.isFk = self.settings["mode"] != 1
        self.isIk = self.settings["mode"] != 0
        self.isFkIk = self.settings["mode"] == 2

        # sample positions along the guide, evenly by arc length ----
        gpos = [datatypes.Vector(p) for p in self.guide.apos]
        self.length = 0.0
        for i in range(len(gpos) - 1):
            self.length += vector.getDistance(gpos[i], gpos[i + 1])
        if self.length < 1e-4:
            self.length = 1.0

        tmp_crv = curve.addCurve(
            self.root, self.getName("tmpSample_crv"), gpos, False, 3
        )
        self.fk_pos = [
            datatypes.Vector(p[0], p[1], p[2])
            for p in curve.get_uniform_world_positions_on_curve(
                tmp_crv.name(), self.fk_number
            )
        ]
        pm.delete(tmp_crv)

        # chain orientation transforms (X down the chain, Y up) -----
        extra = self.fk_pos[-1] + (self.fk_pos[-1] - self.fk_pos[-2])
        self.fk_t = transform.getChainTransform(
            self.fk_pos + [extra], self.normal, self.negate, axis="xy"
        )

        # base space-switch null -------------------------
        self.ik_cns = primitive.addTransform(
            self.root, self.getName("ik_cns"), self.fk_t[0]
        )

        # FK controllers -------------------------------
        self.fk_npo = []
        self.fk_ctl = []
        self.previousTag = self.parentCtlTag
        if self.isFk:
            parent = self.ik_cns
            for i in range(self.fk_number):
                t = self.fk_t[i]
                if i < self.fk_number - 1:
                    dist = vector.getDistance(
                        self.fk_pos[i], self.fk_pos[i + 1]
                    )
                else:
                    dist = vector.getDistance(
                        self.fk_pos[i - 1], self.fk_pos[i]
                    )

                fk_npo = primitive.addTransform(
                    parent, self.getName("fk%s_npo" % i), t
                )
                fk_ctl = self.addCtl(
                    fk_npo,
                    "fk%s_ctl" % i,
                    t,
                    self.color_fk,
                    "cube",
                    w=dist,
                    h=self.size * 0.1,
                    d=self.size * 0.1,
                    po=datatypes.Vector(dist * 0.5 * self.n_factor, 0, 0),
                    tp=self.previousTag,
                )
                attribute.setKeyableAttributes(fk_ctl)
                attribute.setInvertMirror(fk_ctl, ["tx", "ty", "tz"])

                self.fk_npo.append(fk_npo)
                self.fk_ctl.append(fk_ctl)
                self.previousTag = fk_ctl
                parent = fk_ctl

        # IK-spline controls + curves -----------------
        self.ik_ctl = []
        self.ik_npo = []
        if self.isIk:
            ik_pos = [
                self.fk_pos[0],
                vector.linearlyInterpolate(
                    self.fk_pos[0], self.fk_pos[-1], 0.5
                ),
                self.fk_pos[-1],
            ]
            ik_names = ["ik0", "ikMid", "ikTip"]
            prev = self.parentCtlTag
            for j, p in enumerate(ik_pos):
                t = transform.setMatrixPosition(self.fk_t[0], p)
                npo = primitive.addTransform(
                    self.ik_cns, self.getName("%s_npo" % ik_names[j]), t
                )
                ctl = self.addCtl(
                    npo,
                    "%s_ctl" % ik_names[j],
                    t,
                    self.color_ik,
                    "square",
                    w=self.size * 0.25,
                    h=self.size * 0.25,
                    d=self.size * 0.25,
                    ro=datatypes.Vector([0, 0, 1.5708]),
                    tp=prev,
                )
                attribute.setKeyableAttributes(ctl)
                attribute.setInvertMirror(ctl, ["tx", "ry", "rz"])
                self.ik_npo.append(npo)
                self.ik_ctl.append(ctl)
                prev = ctl

            # tangent locators (between base/mid and mid/tip)
            t0 = transform.setMatrixPosition(
                self.fk_t[0],
                vector.linearlyInterpolate(ik_pos[0], ik_pos[2], 0.33),
            )
            self.tan0_loc = primitive.addTransform(
                self.ik_ctl[0], self.getName("tan0_loc"), t0
            )
            t1 = transform.setMatrixPosition(
                self.fk_t[0],
                vector.linearlyInterpolate(ik_pos[0], ik_pos[2], 0.66),
            )
            self.tan1_loc = primitive.addTransform(
                self.ik_ctl[2], self.getName("tan1_loc"), t1
            )

            self.mst_crv = curve.addCnsCurve(
                self.root,
                self.getName("mst_crv"),
                [self.ik_ctl[0], self.tan0_loc, self.tan1_loc, self.ik_ctl[2]],
                3,
            )
            self.slv_crv = curve.addCurve(
                self.root,
                self.getName("slv_crv"),
                [datatypes.Vector()] * 10,
                False,
                3,
            )
            self.mst_crv.setAttr("visibility", False)
            self.slv_crv.setAttr("visibility", False)

            # div_cns chain path-constrained to the slave curve
            self.div_cns = []
            self.twister = []
            self.ref_twist = []
            parentdiv = self.root
            twistRef = primitive.addTransform(
                self.root,
                self.getName("reference"),
                transform.getTransform(self.root),
            )
            t = transform.getTransformLookingAt(
                self.fk_pos[0], self.fk_pos[-1], self.normal, "yx", self.negate
            )
            self.intMRef = primitive.addTransform(
                self.root, self.getName("intMRef"), t
            )
            for i in range(self.fk_number):
                div_cns = primitive.addTransform(
                    parentdiv, self.getName("%s_cns" % i), t
                )
                pm.setAttr(div_cns + ".inheritsTransform", False)
                self.div_cns.append(div_cns)
                parentdiv = div_cns

                tw = primitive.addTransform(
                    twistRef, self.getName("%s_rot_ref" % i), t
                )
                rt = primitive.addTransform(
                    twistRef, self.getName("%s_pos_ref" % i), t
                )
                rt.setTranslation(
                    datatypes.Vector(0.0, 0, 1.0), space="preTransform"
                )
                self.twister.append(tw)
                self.ref_twist.append(rt)

        # Blend layer + bind joints -------------------
        # blnd_npo transforms are FLAT under ik_cns (not chained). Each is
        # parentConstrained (maintainOffset = False, like chain_01) between
        # its FK control and its IK div_cns, so chaining or maintaining an
        # offset would double-transform them.
        self.blnd_npo = []
        self.roll_ref = []
        self.bind_jnt = []
        for i in range(self.fk_number):
            t = self.fk_t[i]
            blnd = primitive.addTransform(
                self.ik_cns, self.getName("blnd%s_npo" % i), t
            )
            roll = primitive.addTransform(
                blnd, self.getName("roll%s" % i), t
            )
            bind_jnt = primitive.addJoint(
                roll, self.getName("bind%s_jnt" % i), t, vis=False
            )
            bind_jnt.attr("drawStyle").set(2)

            self.blnd_npo.append(blnd)
            self.roll_ref.append(roll)
            self.bind_jnt.append(bind_jnt)

        # FK/IK match references ----------------------
        if self.isFkIk:
            self.match_fk = []
            self.match_ik = []
            for i in range(self.fk_number):
                self.match_fk.append(
                    self.add_match_ref(
                        self.fk_ctl[i],
                        self.div_cns[i],
                        "fk%s_mth" % i,
                    )
                )
            for j, name in enumerate(["ik0", "ikMid", "ikTip"]):
                src = [self.fk_ctl[0],
                       self.fk_ctl[self.fk_number // 2],
                       self.fk_ctl[-1]][j]
                self.match_ik.append(
                    self.add_match_ref(
                        self.ik_ctl[j], src, "%s_mth" % name
                    )
                )

        # Ribbon surface -----------------------------
        srf_t = transform.getTransformLookingAt(
            self.fk_pos[0], self.fk_pos[-1], self.normal, "xy", self.negate
        )
        srf_t = transform.setMatrixPosition(
            srf_t,
            vector.linearlyInterpolate(self.fk_pos[0], self.fk_pos[-1], 0.5),
        )
        self.ribbon_root = primitive.addTransform(
            self.root, self.getName("ribbon_root"), srf_t
        )

        width = self.size * 0.2
        self.surface = pm.nurbsPlane(
            name=self.getName("ribbon_srf"),
            width=self.length,
            lengthRatio=width / self.length,
            degree=3,
            patchesU=max(1, self.fk_number - 1),
            patchesV=1,
            axis=[0, 1, 0],
            constructionHistory=False,
        )[0]
        pm.rebuildSurface(
            self.surface,
            constructionHistory=False,
            replaceOriginal=True,
            rebuildType=0,
            keepRange=0,
            degreeU=3,
            degreeV=1,
            spansU=max(1, self.fk_number - 1),
            spansV=1,
            direction=2,
        )
        pm.parent(self.surface, self.ribbon_root, relative=True)
        for at in ("tx", "ty", "tz", "rx", "ry", "rz"):
            pm.setAttr("{}.{}".format(self.surface, at), 0)
        for at in ("sx", "sy", "sz"):
            pm.setAttr("{}.{}".format(self.surface, at), 1)
        # skin drives the surface; the transform must not inherit as well
        pm.setAttr(self.surface + ".inheritsTransform", False)
        attribute.lockAttribute(
            self.surface,
            ["tx", "ty", "tz", "rx", "ry", "rz", "sx", "sy", "sz"],
        )

        self.surface_skin = pm.skinCluster(
            *self.bind_jnt,
            self.surface,
            name=self.getName("ribbon_skinCluster"),
            toSelectedBones=True,
            bindMethod=0,
            skinMethod=0,
            normalizeWeights=1,
            maximumInfluences=2,
            dropoffRate=4,
        )

        # Pin transforms + deform joints -------------
        self.pin_grp = primitive.addTransform(
            self.ribbon_root, self.getName("pins")
        )
        self.pin = []
        self.tweak_ctl = []
        self.attach = []
        self.pin_u = [
            i / (self.jnt_number - 1.0) for i in range(self.jnt_number)
        ]
        self.previousTweakTag = self.parentCtlTag

        for i in range(self.jnt_number):
            pin = primitive.addTransform(
                self.pin_grp, self.getName("pin%s" % i)
            )
            self.pin.append(pin)

            if self.tweak:
                tw_npo = primitive.addTransform(
                    pin, self.getName("tweak%s_npo" % i)
                )
                tw_ctl = self.addCtl(
                    tw_npo,
                    "tweak%s_ctl" % i,
                    transform.getTransform(tw_npo),
                    self.color_ik,
                    "square",
                    w=self.size * 0.12,
                    h=self.size * 0.12,
                    d=self.size * 0.12,
                    tp=self.previousTweakTag,
                )
                attribute.setKeyableAttributes(tw_ctl)
                self.tweak_ctl.append(tw_ctl)
                self.previousTweakTag = tw_ctl
                att_parent = tw_ctl
            else:
                att_parent = pin

            att = primitive.addTransform(
                att_parent, self.getName("attach%s" % i)
            )
            self.attach.append(att)

            self.jnt_pos.append(
                {"obj": att, "name": i, "UniScale": True}
            )

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        """Create the anim and setup rig attributes for the component"""

        if self.isFkIk:
            self.blend_att = self.addAnimParam(
                "blend", "Fk/Ik Blend", "double",
                self.settings["blend"], 0, 1,
            )
            # controls list for the FK/IK tools
            fk_names = [c.name() for c in self.fk_ctl]
            ik_names = [c.name() for c in self.ik_ctl]
            self.ikfk_ctl_att = self.addAnimParam(
                "ctl", "IK/FK Controls", "string",
                ",".join(fk_names + ik_names),
            )

        self.surfaceVis_att = self.addAnimParam(
            "surface_vis", "Surface Vis", "bool", False
        )
        if self.tweak_ctl:
            self.tweakVis_att = self.addAnimParam(
                "tweak_vis", "Tweak Vis", "bool", False
            )

        self.twistStart_att = self.addAnimParam(
            "twist_start", "Twist Start", "double", 0
        )
        self.twistEnd_att = self.addAnimParam(
            "twist_end", "Twist End", "double", 0
        )
        self.roll_att = self.addAnimParam("roll", "Roll", "double", 0)

        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )
        self.maxStretch_att = self.addAnimParam(
            "max_stretch", "Max Stretch", "double", 1.5, 1
        )
        self.maxSquash_att = self.addAnimParam(
            "max_squash", "Max Squash", "double", 0.5, 0, 1
        )

        if self.isIk:
            self.ikPosition_att = self.addAnimParam(
                "ik_position", "IK Position", "double", 0, 0, 1
            )
            self.ikStretch_att = self.addAnimParam(
                "ik_maxstretch", "IK Max Stretch", "double", 1.5, 1
            )
            self.ikSoftness_att = self.addAnimParam(
                "ik_softness", "IK Softness", "double", 0, 0, 1
            )

        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["ikrefarray"].split(",")
            )
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam(
                    "ikref", "Base Ref", 0, ref_names
                )

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        """Create operators and set the relations for the component rig."""

        srf_shape = self.surface.getShape()
        root_dm = node.createDecomposeMatrixNode(
            self.root.attr("worldMatrix[0]")
        )

        # --- IK spline solve --------------------------------------
        if self.isIk:
            # mid control auto-follows base / tip
            mid_cns = pm.pointConstraint(
                self.ik_ctl[0], self.ik_ctl[2], self.ik_npo[1],
                maintainOffset=True,
            )
            pm.orientConstraint(
                self.ik_ctl[0], self.ik_ctl[2], self.ik_npo[1],
                maintainOffset=True,
            )

            op = applyop.gear_curveslide2_op(
                self.slv_crv, self.mst_crv, 0, 1.5, 0.5, 0.5
            )
            pm.connectAttr(self.ikPosition_att, op + ".position")
            pm.connectAttr(self.ikStretch_att, op + ".maxstretch")
            pm.connectAttr(self.ikSoftness_att, op + ".softness")

            for i in range(self.fk_number):
                u = i / (self.fk_number - 1.0)
                if i == 0:
                    u = (1.0 / (self.fk_number - 1.0)) / 10.0
                if u >= 1.0:
                    u = 0.99

                cns = applyop.pathCns(
                    self.div_cns[i], self.slv_crv, False, u, True
                )
                cns.setAttr("frontAxis", 0)  # X down the length
                cns.setAttr("upAxis", 1)     # Y up

                intMatrix = applyop.gear_intmatrix_op(
                    self.ik_ctl[0] + ".worldMatrix",
                    self.ik_ctl[2] + ".worldMatrix",
                    u,
                )
                dm = node.createDecomposeMatrixNode(intMatrix + ".output")
                pm.connectAttr(
                    dm + ".outputRotate", self.twister[i].attr("rotate")
                )
                pm.parentConstraint(
                    self.twister[i], self.ref_twist[i], maintainOffset=True
                )
                pm.connectAttr(
                    self.ref_twist[i] + ".translate",
                    cns + ".worldUpVector",
                )

        # --- blend the bind layer between FK and IK ---------------
        for i in range(self.fk_number):
            blnd = self.blnd_npo[i]
            if self.isFkIk:
                rev = node.createReverseNode(self.blend_att)
                cns = pm.parentConstraint(
                    self.fk_ctl[i], self.div_cns[i], blnd,
                    maintainOffset=False,
                )
                cns.interpType.set(0)
                w = pm.parentConstraint(cns, query=True, weightAliasList=True)
                pm.connectAttr(rev + ".outputX", "{}.{}".format(cns, w[0]))
                pm.connectAttr(self.blend_att, "{}.{}".format(cns, w[1]))
            elif self.isIk:
                pm.parentConstraint(
                    self.div_cns[i], blnd, maintainOffset=False
                )
            else:  # FK only
                pm.parentConstraint(
                    self.fk_ctl[i], blnd, maintainOffset=False
                )

        # --- twist / roll on the roll references ------------------
        for i, roll in enumerate(self.roll_ref):
            u = i / (self.fk_number - 1.0)
            blend = node.createBlendNode(
                [self.twistEnd_att], [self.twistStart_att], u
            )
            add = node.createPlusMinusAverage1D(
                [blend + ".outputR", self.roll_att]
            )
            pm.connectAttr(add + ".output1D", roll + ".rotateX")

        # --- surface attach via pointOnSurfaceInfo ----------------
        for i, pin in enumerate(self.pin):
            u = self.pin_u[i]

            posi = pm.createNode(
                "pointOnSurfaceInfo",
                name=self.getName("pin%s_posi" % i),
            )
            posi.attr("turnOnPercentage").set(False)
            posi.attr("parameterU").set(u)
            posi.attr("parameterV").set(0.5)
            pm.connectAttr(
                srf_shape.attr("worldSpace[0]"), posi.attr("inputSurface")
            )

            mtx = pm.createNode(
                "fourByFourMatrix", name=self.getName("pin%s_m4x4" % i)
            )
            pm.connectAttr(posi.attr("normalizedTangentUX"), mtx.attr("in00"))
            pm.connectAttr(posi.attr("normalizedTangentUY"), mtx.attr("in01"))
            pm.connectAttr(posi.attr("normalizedTangentUZ"), mtx.attr("in02"))
            pm.connectAttr(posi.attr("normalizedNormalX"), mtx.attr("in10"))
            pm.connectAttr(posi.attr("normalizedNormalY"), mtx.attr("in11"))
            pm.connectAttr(posi.attr("normalizedNormalZ"), mtx.attr("in12"))
            pm.connectAttr(posi.attr("normalizedTangentVX"), mtx.attr("in20"))
            pm.connectAttr(posi.attr("normalizedTangentVY"), mtx.attr("in21"))
            pm.connectAttr(posi.attr("normalizedTangentVZ"), mtx.attr("in22"))
            pm.connectAttr(posi.attr("positionX"), mtx.attr("in30"))
            pm.connectAttr(posi.attr("positionY"), mtx.attr("in31"))
            pm.connectAttr(posi.attr("positionZ"), mtx.attr("in32"))

            mm = applyop.gear_mulmatrix_op(
                mtx.attr("output"),
                pin.attr("parentInverseMatrix[0]"),
            )
            dm = node.createDecomposeMatrixNode(mm + ".output")
            pm.connectAttr(dm + ".outputTranslate", pin.attr("translate"))
            pm.connectAttr(dm + ".outputRotate", pin.attr("rotate"))

        # --- volume / squash & stretch ---------------------------
        dist = node.createDistNode(self.bind_jnt[0], self.bind_jnt[-1])
        rest_len = pm.getAttr(dist + ".distance")
        cur_len = node.createDivNode(
            dist + ".distance", root_dm + ".outputScaleX"
        )
        ratio = node.createDivNode(cur_len + ".outputX", rest_len)
        clamp = node.createClampNode(
            [ratio + ".outputX", 0, 0],
            [self.maxSquash_att, 0, 0],
            [self.maxStretch_att, 0, 0],
        )
        inv = node.createDivNode(1.0, clamp + ".outputR")
        vol_factor = node.createPowNode(inv + ".outputX", 0.5)
        vol_blend = node.createBlendNode(
            [1.0, 1.0, 1.0],
            [vol_factor + ".outputX"] * 3,
            self.volume_att,
        )
        for att in self.attach:
            mul = node.createMulNode(
                [
                    root_dm + ".outputScaleX",
                    root_dm + ".outputScaleY",
                    root_dm + ".outputScaleZ",
                ],
                [1.0, vol_blend + ".outputR", vol_blend + ".outputG"],
            )
            pm.connectAttr(mul + ".output", att.attr("scale"))

        # --- visibilities ---------------------------------------
        pm.connectAttr(
            self.surfaceVis_att, self.surface.attr("visibility")
        )
        if self.isFkIk:
            fkvis = node.createReverseNode(self.blend_att)
            for ctl in self.fk_ctl:
                for shp in ctl.getShapes():
                    pm.connectAttr(fkvis + ".outputX", shp.attr("visibility"))
            for ctl in self.ik_ctl:
                for shp in ctl.getShapes():
                    pm.connectAttr(
                        self.blend_att, shp.attr("visibility")
                    )
        if self.tweak_ctl:
            for ctl in self.tweak_ctl:
                for shp in ctl.getShapes():
                    pm.connectAttr(
                        self.tweakVis_att, shp.attr("visibility")
                    )

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        """Set the relation beetween object from guide to rig"""

        if self.isFk:
            base_ctl = self.fk_ctl[0]
            tip_ctl = self.fk_ctl[-1]
        else:
            base_ctl = self.ik_ctl[0]
            tip_ctl = self.ik_ctl[-1]

        self.relatives["root"] = base_ctl
        self.controlRelatives["root"] = base_ctl
        self.jointRelatives["root"] = 0
        self.aliasRelatives["root"] = "base"

        for i in range(len(self.guide.apos) - 1):
            self.relatives["%s_loc" % i] = tip_ctl
            self.controlRelatives["%s_loc" % i] = tip_ctl
            self.jointRelatives["%s_loc" % i] = min(
                i + 1, len(self.jnt_pos) - 1
            )
            self.aliasRelatives["%s_loc" % i] = "tip"

    def addConnection(self):
        self.connections["standard"] = self.connect_standard
        self.connections["orientation"] = self.connect_orientation

    def connect_orientation(self):
        self.connect_orientCns()

    def connect_standard(self):
        self.connect_standardWithSimpleIkRef()


#############################################
# FK / IK MATCH  (module level, callable from a picker / menu)
#############################################

def ribbon_IKToFK(fk_controls, ik_controls):
    """Snap the IK-spline controls to match the current FK pose.

    Args:
        fk_controls (list): fk control names, base -> tip
        ik_controls (list): [ik0_ctl, ikMid_ctl, ikTip_ctl]
    """
    import mgear.pymaya as pm
    from mgear.core import attribute

    fk = [pm.PyNode(x) for x in fk_controls]
    ik = [pm.PyNode(x) for x in ik_controls]
    mats = {n.name(): n.getAttr("worldMatrix") for n in fk}

    attribute.reset_SRT(ik)
    ik[0].setMatrix(mats[fk[0].name()], worldSpace=True)
    ik[-1].setMatrix(mats[fk[-1].name()], worldSpace=True)
    ik[1].setMatrix(mats[fk[len(fk) // 2].name()], worldSpace=True)


def ribbon_FKToIK(fk_controls, ik_controls=None):
    """Snap the FK controls to match the current IK-spline result.

    Reads the ``*_mth`` match transforms wired on each FK control.

    Args:
        fk_controls (list): fk control names, base -> tip
        ik_controls: unused, kept for signature symmetry.
    """
    import mgear.pymaya as pm
    from mgear.core import attribute

    fk = [pm.PyNode(x) for x in fk_controls]
    refs = []
    for ctl in fk:
        if ctl.hasAttr("match_ref"):
            cnx = ctl.match_ref.listConnections()
            refs.append(cnx[0] if cnx else None)
        else:
            refs.append(None)
    mats = [r.getAttr("worldMatrix") if r else None for r in refs]

    attribute.reset_SRT(fk)
    for ctl, m in zip(fk, mats):
        if m is not None:
            ctl.setMatrix(m, worldSpace=True)
