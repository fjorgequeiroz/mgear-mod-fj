"""Component Ribbon 01 module.

A ribbon / surface rig - the classic follicle-style method (antCGI
"Rigging In Maya - Part 18 - Ribbons"), using ``pointOnSurfaceInfo`` +
``fourByFourMatrix`` (the mGear "rivet" pattern) for the surface attach.

Pipeline
--------
1. A NURBS plane is laid along the guide, rebuilt to degree 3 along the
   length (U) / degree 1 across the width (V), one U span per FK segment,
   0-1 parameter range.
2. A real, parented FK chain drives one hidden **bind joint** per control.
   The surface is skinned to those bind joints.
3. ``jntNb`` **pin transforms** are attached to the surface at evenly
   spaced U values (V = 0.5) via ``pointOnSurfaceInfo`` -> matrix ->
   ``offsetParentMatrix``. Each pin carries a deform joint, plus an
   optional tweak control.
4. Extra animation features on the UI host:
   * **Twist Start / Twist End** - roll distributed along the length,
     applied to the bind joints so the skin twists and the pins inherit.
   * **Roll** - uniform roll of the whole ribbon.
   * **Volume** + **Squash / Stretch clamps** - distance-driven volume
     preservation on the deform joints.

No cycle: fk_ctl -> skinCluster -> surface -> pointOnSurfaceInfo -> pin ->
joint, one direction. The volume driver is the distance between the
FK-driven bind joints, never the deformed surface.
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

        # FK controllers + bind joints ------------------
        # A plain parented FK chain. Nothing fancy.
        self.fk_npo = []
        self.fk_ctl = []
        self.bind_jnt = []
        self.roll_ref = []
        self.previousTag = self.parentCtlTag
        parent = self.ik_cns
        for i in range(self.fk_number):
            t = self.fk_t[i]
            if i < self.fk_number - 1:
                dist = vector.getDistance(self.fk_pos[i], self.fk_pos[i + 1])
            else:
                dist = vector.getDistance(self.fk_pos[i - 1], self.fk_pos[i])

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

            # roll pivot: a child that will receive the twist rotation so
            # the fk_ctl channels stay clean
            roll = primitive.addTransform(
                fk_ctl, self.getName("roll%s" % i), t
            )
            bind_jnt = primitive.addJoint(
                roll, self.getName("bind%s_jnt" % i), t, vis=False
            )
            bind_jnt.attr("drawStyle").set(2)

            self.fk_npo.append(fk_npo)
            self.fk_ctl.append(fk_ctl)
            self.bind_jnt.append(bind_jnt)
            self.roll_ref.append(roll)

            self.previousTag = fk_ctl
            parent = fk_ctl

        # Ribbon surface --------------------------------
        # ribbon_root is oriented on the guide. The surface is parented in
        # with a zeroed local transform, then deformed only by the
        # skinCluster. The pins are driven by world-space matrices into
        # their offsetParentMatrix, so ribbon_root's transform never
        # matters to them.
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

        # parent the plane onto the oriented ribbon_root and zero its local
        pm.parent(self.surface, self.ribbon_root, relative=True)
        for at in ("tx", "ty", "tz", "rx", "ry", "rz"):
            pm.setAttr("{}.{}".format(self.surface, at), 0)
        for at in ("sx", "sy", "sz"):
            pm.setAttr("{}.{}".format(self.surface, at), 1)
        attribute.lockAttribute(
            self.surface,
            ["tx", "ty", "tz", "rx", "ry", "rz", "sx", "sy", "sz"],
        )

        # skin the surface to the bind joints
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

        # Pin transforms + deform joints ---------------
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

        self.surfaceVis_att = self.addAnimParam(
            "surface_vis", "Surface Vis", "bool", False
        )
        if self.tweak_ctl:
            self.tweakVis_att = self.addAnimParam(
                "tweak_vis", "Tweak Vis", "bool", False
            )

        # twist / roll
        self.twistStart_att = self.addAnimParam(
            "twist_start", "Twist Start", "double", 0
        )
        self.twistEnd_att = self.addAnimParam(
            "twist_end", "Twist End", "double", 0
        )
        self.roll_att = self.addAnimParam("roll", "Roll", "double", 0)

        # volume / squash & stretch
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )
        self.maxStretch_att = self.addAnimParam(
            "max_stretch", "Max Stretch", "double", 1.5, 1
        )
        self.maxSquash_att = self.addAnimParam(
            "max_squash", "Max Squash", "double", 0.5, 0, 1
        )

        # base space switch
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

        # --- twist / roll on the bind-joint roll references ----------
        # The length axis is local X of the chain transforms. Each roll
        # reference gets twist_start at u=0 blended to twist_end at u=1,
        # plus the uniform Roll.
        for i, roll in enumerate(self.roll_ref):
            u = i / (self.fk_number - 1.0)
            # blendColors: blender=1 -> color1, blender=0 -> color2.
            # u=0 must give twist_start, u=1 must give twist_end.
            blend = node.createBlendNode(
                [self.twistEnd_att], [self.twistStart_att], u
            )
            add = node.createPlusMinusAverage1D(
                [blend + ".outputR", self.roll_att]
            )
            pm.connectAttr(add + ".output1D", roll + ".rotateX")

        # --- surface attach via pointOnSurfaceInfo -------------------
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
            # X = U tangent (length)
            pm.connectAttr(posi.attr("normalizedTangentUX"), mtx.attr("in00"))
            pm.connectAttr(posi.attr("normalizedTangentUY"), mtx.attr("in01"))
            pm.connectAttr(posi.attr("normalizedTangentUZ"), mtx.attr("in02"))
            # Y = surface normal
            pm.connectAttr(posi.attr("normalizedNormalX"), mtx.attr("in10"))
            pm.connectAttr(posi.attr("normalizedNormalY"), mtx.attr("in11"))
            pm.connectAttr(posi.attr("normalizedNormalZ"), mtx.attr("in12"))
            # Z = V tangent (width)
            pm.connectAttr(posi.attr("normalizedTangentVX"), mtx.attr("in20"))
            pm.connectAttr(posi.attr("normalizedTangentVY"), mtx.attr("in21"))
            pm.connectAttr(posi.attr("normalizedTangentVZ"), mtx.attr("in22"))
            # position
            pm.connectAttr(posi.attr("positionX"), mtx.attr("in30"))
            pm.connectAttr(posi.attr("positionY"), mtx.attr("in31"))
            pm.connectAttr(posi.attr("positionZ"), mtx.attr("in32"))

            # bring the world matrix into the pin's parent space, then
            # split into translate / rotate (unambiguous - no reliance on
            # offsetParentMatrix space or inheritsTransform).
            mm = applyop.gear_mulmatrix_op(
                mtx.attr("output"),
                pin.attr("parentInverseMatrix[0]"),
            )
            dm = node.createDecomposeMatrixNode(mm + ".output")
            pm.connectAttr(dm + ".outputTranslate", pin.attr("translate"))
            pm.connectAttr(dm + ".outputRotate", pin.attr("rotate"))

        # --- volume / squash & stretch ------------------------------
        # driver = distance between first and last bind joints
        dist = node.createDistNode(self.bind_jnt[0], self.bind_jnt[-1])
        rest_len = pm.getAttr(dist + ".distance")
        # normalise out the global rig scale
        cur_len = node.createDivNode(
            dist + ".distance", root_dm + ".outputScaleX"
        )
        ratio = node.createDivNode(cur_len + ".outputX", rest_len)
        # clamp the stretch/squash range
        clamp = node.createClampNode(
            [ratio + ".outputX", 0, 0],
            [self.maxSquash_att, 0, 0],
            [self.maxStretch_att, 0, 0],
        )
        # cross-section factor = 1 / sqrt(len ratio) for volume preservation
        inv = node.createDivNode(1.0, clamp + ".outputR")
        vol_factor = node.createPowNode(inv + ".outputX", 0.5)
        # blend towards 1.0 by the Volume attribute
        vol_blend = node.createBlendNode(
            [1.0, 1.0, 1.0],
            [vol_factor + ".outputX"] * 3,
            self.volume_att,
        )

        for att in self.attach:
            # X follows the global rig scale only (length); Y and Z also get
            # the cross-section volume factor
            mul = node.createMulNode(
                [
                    root_dm + ".outputScaleX",
                    root_dm + ".outputScaleY",
                    root_dm + ".outputScaleZ",
                ],
                [1.0, vol_blend + ".outputR", vol_blend + ".outputG"],
            )
            pm.connectAttr(mul + ".output", att.attr("scale"))

        # --- visibilities ------------------------------------------
        pm.connectAttr(
            self.surfaceVis_att, self.surface.attr("visibility")
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

        self.relatives["root"] = self.fk_ctl[0]
        self.controlRelatives["root"] = self.fk_ctl[0]
        self.jointRelatives["root"] = 0
        self.aliasRelatives["root"] = "base"

        last_fk = len(self.fk_ctl) - 1
        for i in range(len(self.guide.apos) - 1):
            self.relatives["%s_loc" % i] = self.fk_ctl[min(i + 1, last_fk)]
            self.controlRelatives["%s_loc" % i] = self.fk_ctl[
                min(i + 1, last_fk)
            ]
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
