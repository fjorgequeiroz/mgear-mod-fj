"""Component Ribbon 01 module.

A ribbon / surface rig - the antCGI "Rigging In Maya - Part 18 - Ribbons"
method, with the follicles swapped for a single ``uvPin`` node.

Pipeline
--------
1. A NURBS surface is lofted from two rail curves offset across the width
   through the FK positions, then rebuilt to degree 3 along the length (U),
   degree 1 across the width (V), one U span per FK segment, 0-1 range.
2. The surface is skinned to a chain of **bind joints**, one per FK
   controller. Each FK controller drives its bind joint; a middle
   controller auto-follows its neighbours (``Mid Follow`` attribute).
3. A single ``uvPin`` node drives ``jntNb`` **pin transforms** from fixed
   UVs (evenly spaced U, V = 0.5). Each pin carries one deform joint, plus
   an optional tweak control.
4. Twisting the end controller around the length axis twists the surface,
   and the pins inherit it for free. A distance-driven factor gives an
   optional squash & stretch on the cross-section (``Volume`` attribute).

Settings
--------
* **FK Controllers** (``fkNb``) - number of FK controls / bind joints.
* **Deform Joints** (``jntNb``) - number of pins / deform joints.
* **Tweak Controls** - add a tweak control on every pin.

No cycle: fk_ctl -> skinCluster -> surface -> uvPin -> pin -> joint, one
direction. The squash driver is the distance between the FK-driven bind
joints, never the deformed surface.
"""

import mgear.pymaya as pm
from mgear.pymaya import datatypes

from mgear.shifter import component

from mgear.core import node, vector, curve
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
        self.up_axis = "y"

        # sample positions along the guide, evenly by arc length ----
        gpos = [datatypes.Vector(p) for p in self.guide.apos]
        self.length = 0.0
        for i in range(len(gpos) - 1):
            self.length += vector.getDistance(gpos[i], gpos[i + 1])

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

        # base space-switch null (target of the ikrefarray connector)
        self.ik_cns = primitive.addTransform(
            self.root, self.getName("ik_cns"), self.fk_t[0]
        )

        # FK controllers + bind joints -------------------
        self.fk_npo = []
        self.fk_ctl = []
        self.mid_npo = []
        self.bind_jnt = []
        self.previousTag = self.parentCtlTag
        parent = self.ik_cns
        for i in range(self.fk_number):
            t = self.fk_t[i]
            if i < self.fk_number - 1:
                dist = vector.getDistance(self.fk_pos[i], self.fk_pos[i + 1])
            else:
                dist = vector.getDistance(self.fk_pos[i - 1], self.fk_pos[i])

            # a middle FK control is not parented under the previous one, so
            # it can be driven by an auto-follow constraint (see addOperators)
            is_middle = 0 < i < self.fk_number - 1
            ctl_parent = self.ik_cns if is_middle else parent

            fk_npo = primitive.addTransform(
                ctl_parent, self.getName("fk%s_npo" % i), t
            )
            fk_ctl = self.addCtl(
                fk_npo,
                "fk%s_ctl" % i,
                t,
                self.color_fk,
                "square" if is_middle else "cube",
                w=dist,
                h=self.size * 0.1,
                d=self.size * 0.1,
                po=datatypes.Vector(dist * 0.5 * self.n_factor, 0, 0),
                tp=self.previousTag,
            )
            attribute.setKeyableAttributes(fk_ctl)
            attribute.setInvertMirror(fk_ctl, ["tx", "ty", "tz"])

            # bind joint the surface is skinned to (hidden, never drawn)
            bind_jnt = primitive.addJoint(
                fk_ctl, self.getName("bind%s_jnt" % i), t, vis=False
            )
            bind_jnt.attr("drawStyle").set(2)

            self.fk_npo.append(fk_npo)
            self.fk_ctl.append(fk_ctl)
            self.bind_jnt.append(bind_jnt)
            if is_middle:
                self.mid_npo.append((i, fk_npo))
            self.previousTag = fk_ctl
            if not is_middle:
                parent = fk_ctl

        # Ribbon surface --------------------------------
        # Built by lofting two curves offset across the width, so the U
        # parameter is guaranteed to run along the length of the chain and
        # V across the width (no reliance on nurbsPlane's axis convention).
        self.ribbon_root = primitive.addTransform(
            self.root,
            self.getName("ribbon_root"),
            transform.getTransform(self.root),
        )

        half_w = self.size * 0.1
        rail_a = []
        rail_b = []
        for i, m in enumerate(self.fk_t[: self.fk_number]):
            # local Z of each chain transform is the width direction
            z_axis = datatypes.Vector(m[2][0], m[2][1], m[2][2]).normal()
            rail_a.append(self.fk_pos[i] + z_axis * half_w)
            rail_b.append(self.fk_pos[i] - z_axis * half_w)

        crv_a = curve.addCurve(
            self.ribbon_root, self.getName("ribbonRailA_crv"), rail_a,
            False, 3,
        )
        crv_b = curve.addCurve(
            self.ribbon_root, self.getName("ribbonRailB_crv"), rail_b,
            False, 3,
        )
        self.surface = pm.loft(
            crv_a, crv_b,
            name=self.getName("ribbon_srf"),
            constructionHistory=False,
            uniform=True,
            degree=1,
            sectionSpans=1,
            range=False,
            polygon=0,
        )[0]
        pm.delete(crv_a, crv_b)

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
        pm.parent(self.surface, self.ribbon_root)
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

        # uvPin attach + deform joints -----------------
        # One uvPin drives all the attach transforms from fixed UVs; the
        # skinCluster deforms the surface and the pins follow.
        self.pin_grp = primitive.addTransform(
            self.ribbon_root, self.getName("pins"),
            transform.getTransform(self.ribbon_root),
        )
        self.pin_tr = []
        self.pin_npo = []
        self.tweak_ctl = []
        self.attach = []
        self.tweak = self.settings["tweakControls"]
        self.previousTweakTag = self.parentCtlTag

        # uvPin needs the deformed shape and an undeformed reference shape.
        # We duplicate the surface shape as the "original" and feed the live
        # skinned shape as the "deformed".
        srf_shape = self.surface.getShape()
        orig_srf = pm.duplicate(
            self.surface, name=self.getName("ribbon_srfOrig")
        )[0]
        pm.parent(orig_srf, self.ribbon_root)
        pm.setAttr(orig_srf + ".visibility", False)
        attribute.lockAttribute(
            orig_srf,
            ["tx", "ty", "tz", "rx", "ry", "rz", "sx", "sy", "sz"],
        )
        orig_shape = orig_srf.getShape()

        self.uvpin = pm.createNode(
            "uvPin", name=self.getName("ribbon_uvPin")
        )
        # X = U tangent (down the length), Y = surface normal, Z = width
        pm.setAttr(self.uvpin + ".tangentAxis", 0)
        pm.setAttr(self.uvpin + ".normalAxis", 1)
        pm.connectAttr(
            srf_shape.attr("worldSpace[0]"),
            self.uvpin + ".deformedGeometry",
        )
        pm.connectAttr(
            orig_shape.attr("worldSpace[0]"),
            self.uvpin + ".originalGeometry",
        )
        # output relative to the pin group so the pins can be simple children
        pm.setAttr(self.uvpin + ".relativeSpaceMode", 1)
        pm.connectAttr(
            self.pin_grp.attr("worldInverseMatrix[0]"),
            self.uvpin + ".relativeSpaceMatrix",
        )

        for i in range(self.jnt_number):
            frac = i / (self.jnt_number - 1.0)
            pm.setAttr(self.uvpin + ".coordinate[%s].coordinateU" % i, frac)
            pm.setAttr(self.uvpin + ".coordinate[%s].coordinateV" % i, 0.5)

            pin_npo = primitive.addTransform(
                self.pin_grp, self.getName("pin%s_npo" % i)
            )
            pin_tr = primitive.addTransform(
                pin_npo, self.getName("pin%s" % i)
            )
            pm.connectAttr(
                self.uvpin + ".outputMatrix[%s]" % i,
                pin_tr + ".offsetParentMatrix",
            )

            self.pin_npo.append(pin_npo)
            self.pin_tr.append(pin_tr)

            # optional tweak control
            if self.tweak:
                tw_npo = primitive.addTransform(
                    pin_tr, self.getName("tweak%s_npo" % i)
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
                jnt_parent = tw_ctl
            else:
                jnt_parent = pin_tr

            att = primitive.addTransform(
                jnt_parent, self.getName("attach%s" % i),
                transform.getTransform(jnt_parent),
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
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )
        if self.fk_number > 2:
            self.midFollow_att = self.addAnimParam(
                "mid_follow", "Mid Follow", "double", 1, 0, 1
            )
        if self.tweak_ctl:
            self.tweakVis_att = self.addAnimParam(
                "tweak_vis", "Tweak Vis", "bool", False
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

        dm_scl = node.createDecomposeMatrixNode(
            self.root.attr("worldMatrix[0]")
        )

        # --- middle FK controls auto-follow their neighbours ----------
        for idx, npo in self.mid_npo:
            prev_ctl = self.fk_ctl[idx - 1]
            next_ctl = self.fk_ctl[idx + 1]
            cns = pm.parentConstraint(
                prev_ctl, next_ctl, npo, maintainOffset=True
            )
            cns.interpType.set(2)  # shortest
            w = pm.parentConstraint(cns, query=True, weightAliasList=True)
            for wa in w:
                pm.connectAttr(self.midFollow_att, "{}.{}".format(cns, wa))

        # --- squash & stretch volume ------------------------------
        # length driver = distance between the first and last bind joints
        # (FK driven, so no cycle with the squash output)
        dist = node.createDistNode(self.bind_jnt[0], self.bind_jnt[-1])
        rest_len = pm.getAttr(dist + ".distance")
        len_scaled = node.createDivNode(
            dist + ".distance", dm_scl + ".outputScaleX"
        )
        # cross-section squash factor = sqrt(rest / current) for rough
        # volume preservation, blended by the volume attribute
        squash_ratio = node.createDivNode(rest_len, len_scaled + ".outputX")
        squash_pow = pm.createNode("multiplyDivide")
        pm.setAttr(squash_pow + ".operation", 3)  # power
        pm.connectAttr(squash_ratio + ".outputX", squash_pow + ".input1X")
        pm.setAttr(squash_pow + ".input2X", 0.5)
        # volume 0 -> factor 1.0 (no squash), volume 1 -> factor sqrt(ratio)
        vol_blend = node.createBlendNode(
            [1.0, 1.0, 1.0],
            [squash_pow + ".outputX"] * 3,
            self.volume_att,
        )

        for att in self.attach:
            # X follows the length (global scale only); Y and Z get the
            # cross-section squash on top of the global scale
            mul = node.createMulNode(
                [
                    dm_scl + ".outputScaleX",
                    dm_scl + ".outputScaleY",
                    dm_scl + ".outputScaleZ",
                ],
                [1.0, vol_blend + ".outputR", vol_blend + ".outputG"],
            )
            pm.connectAttr(mul + ".output", att.attr("s"))

        # --- visibilities ----------------------------------------
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
