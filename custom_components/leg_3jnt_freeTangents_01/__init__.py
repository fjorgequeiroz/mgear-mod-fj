"""Component Leg 3 joints Free Tangents 01 module.

3 bones leg (quadrupeds / animals) built on top of the standard
leg_3jnt_01 dual IK/FK solver (unchanged), with the deformation backbone
replaced by the free-tangent twist system ported from
leg_2jnt_freeTangents_01: spline-IK twist chains, shaped by per-segment
tangent controls, instead of the roundnessKnee / roundnessAnkle bulge
attributes and the gear_rollsplinekine_op division loop.

Because there are 3 bones (upleg, midleg, lowleg) and 2 junctions (knee,
ankle) instead of the reference's 2 bones / 1 junction, the reference's
single-junction tangent pattern (tws1A/tws1B under mid_ctl, kneeTangent_ctl
under mid_ctl, uplegTwistChain/lowlegTwistChain) is mirrored twice: once at
the knee (tws1A/tws1B under knee_ctl, kneeTangent_ctl under knee_ctl) and
once at the ankle (tws2A/tws2B under ankle_ctl, ankleTangent_ctl under
ankle_ctl).
"""

import mgear.pymaya as pm
from mgear.pymaya import datatypes

from mgear.shifter import component

from mgear.core import node, fcurve, applyop, vector, icon
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

        self.setup = primitive.addTransformFromPos(
            self.setupWS, self.getName("WS")
        )
        attribute.lockAttribute(self.setup)

        self.WIP = self.options["mode"]

        self.normal = self.getNormalFromPos(self.guide.apos)

        self.length0 = vector.getDistance(
            self.guide.apos[0], self.guide.apos[1]
        )
        self.length1 = vector.getDistance(
            self.guide.apos[1], self.guide.apos[2]
        )
        self.length2 = vector.getDistance(
            self.guide.apos[2], self.guide.apos[3]
        )
        self.length3 = vector.getDistance(
            self.guide.apos[3], self.guide.apos[4]
        )

        # 3bones chain
        self.chain3bones = primitive.add2DChain(
            self.setup,
            self.getName("chain3bones%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            self.WIP,
        )

        # 2bones chain
        self.chain2bones = primitive.add2DChain(
            self.setup,
            self.getName("chain2bones%s_jnt"),
            self.guide.apos[0:3],
            self.normal,
            False,
            self.WIP,
        )

        # Leg chain
        self.legBones = primitive.add2DChain(
            self.root,
            self.getName("legBones%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            True,
        )
        if not self.WIP:
            for b in self.legBones:
                b.attr("drawStyle").set(2)

        # Leg chain FK ref
        self.legBonesFK = primitive.add2DChain(
            self.root,
            self.getName("legFK%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            self.WIP,
        )

        # Leg chain IK ref
        self.legBonesIK = primitive.add2DChain(
            self.root,
            self.getName("legIK%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            self.WIP,
        )

        # 1 bone chain for upv ref
        self.legChainUpvRef = primitive.add2DChain(
            self.root,
            self.getName("legUpvRef%s_jnt"),
            [self.guide.apos[0], self.guide.apos[3]],
            self.normal,
            False,
            self.WIP,
        )

        # mid joints
        self.mid1_jnt = primitive.addJoint(
            self.legBones[0],
            self.getName("mid1_jnt"),
            self.legBones[1].getMatrix(worldSpace=True),
            self.WIP,
        )

        self.mid1_jnt.attr("radius").set(3)
        self.mid1_jnt.setAttr("jointOrient", 0, 0, 0)

        self.mid2_jnt = primitive.addJoint(
            self.legBones[1],
            self.getName("mid2_jnt"),
            self.legBones[2].getMatrix(worldSpace=True),
            self.WIP,
        )

        self.mid2_jnt.attr("radius").set(3)
        self.mid2_jnt.setAttr("jointOrient", 0, 0, 0)

        # base Controlers -----------------------------------
        t = transform.getTransformFromPos(self.guide.apos[0])
        self.root_npo = primitive.addTransform(
            self.root, self.getName("root_npo"), t
        )

        self.root_ctl = self.addCtl(
            self.root_npo,
            "root_ctl",
            t,
            self.color_fk,
            "circle",
            w=self.length0 / 6,
            tp=self.parentCtlTag,
        )
        attribute.lockAttribute(self.root_ctl, ["sx", "sy", "sz", "v"])

        # FK Controlers -----------------------------------
        t = transform.getTransformLookingAt(
            self.guide.apos[0],
            self.guide.apos[1],
            self.normal,
            "xz",
            self.negate,
        )

        self.fk0_npo = primitive.addTransform(
            self.root_ctl, self.getName("fk0_npo"), t
        )

        self.fk0_ctl = self.addCtl(
            self.fk0_npo,
            "fk0_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length0,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length0 * self.n_factor, 0, 0),
            tp=self.root_ctl,
        )
        attribute.setKeyableAttributes(self.fk0_ctl)

        t = transform.getTransformLookingAt(
            self.guide.apos[1],
            self.guide.apos[2],
            self.normal,
            "xz",
            self.negate,
        )
        self.fk1_npo = primitive.addTransform(
            self.fk0_ctl, self.getName("fk1_npo"), t
        )
        self.fk1_ctl = self.addCtl(
            self.fk1_npo,
            "fk1_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length1,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length1 * self.n_factor, 0, 0),
            tp=self.fk0_ctl,
        )
        attribute.setKeyableAttributes(self.fk1_ctl)

        t = transform.getTransformLookingAt(
            self.guide.apos[2],
            self.guide.apos[3],
            self.normal,
            "xz",
            self.negate,
        )

        self.fk2_npo = primitive.addTransform(
            self.fk1_ctl, self.getName("fk2_npo"), t
        )

        self.fk2_ctl = self.addCtl(
            self.fk2_npo,
            "fk2_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length2,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length2 * self.n_factor, 0, 0),
            tp=self.fk1_ctl,
        )

        attribute.setKeyableAttributes(self.fk2_ctl)

        t = transform.getTransformLookingAt(
            self.guide.apos[3],
            self.guide.apos[4],
            self.normal,
            "xz",
            self.negate,
        )

        self.fk3_npo = primitive.addTransform(
            self.fk2_ctl, self.getName("fk3_npo"), t
        )

        self.fk3_ctl = self.addCtl(
            self.fk3_npo,
            "fk3_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length3,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length3 * self.n_factor, 0, 0),
            tp=self.fk2_ctl,
        )

        attribute.setKeyableAttributes(self.fk3_ctl)

        self.fk_ctl = [self.fk0_ctl, self.fk1_ctl, self.fk2_ctl, self.fk3_ctl]

        for x in self.fk_ctl:
            attribute.setInvertMirror(x, ["tx", "ty", "tz"])

        # Mid Controlers ------------------------------------
        # knee_lvl follows the mid1 joint; knee_cns is the pin switch layer
        # (Auto = knee_lvl, or a pin reference).
        self.knee_lvl = primitive.addTransform(
            self.root,
            self.getName("knee_lvl"),
            transform.getTransform(self.mid1_jnt),
        )
        self.knee_cns = primitive.addTransform(
            self.knee_lvl,
            self.getName("knee_cns"),
            transform.getTransform(self.mid1_jnt),
        )

        self.knee_ctl = self.addCtl(
            self.knee_cns,
            "knee_ctl",
            transform.getTransform(self.mid1_jnt),
            self.color_ik,
            "sphere",
            w=self.size * 0.2,
            tp=self.root_ctl,
        )

        attribute.setInvertMirror(self.knee_ctl, ["tx", "ty", "tz"])
        attribute.lockAttribute(self.knee_ctl, ["sx", "sy", "sz", "v"])

        self.ankle_lvl = primitive.addTransform(
            self.root,
            self.getName("ankle_lvl"),
            transform.getTransform(self.mid2_jnt),
        )
        self.ankle_cns = primitive.addTransform(
            self.ankle_lvl,
            self.getName("ankle_cns"),
            transform.getTransform(self.mid2_jnt),
        )

        self.ankle_ctl = self.addCtl(
            self.ankle_cns,
            "ankle_ctl",
            transform.getTransform(self.mid2_jnt),
            self.color_ik,
            "sphere",
            w=self.size * 0.2,
            tp=self.knee_ctl,
        )

        attribute.setInvertMirror(self.ankle_ctl, ["tx", "ty", "tz"])
        attribute.lockAttribute(self.ankle_ctl, ["sx", "sy", "sz", "v"])

        # IK controls --------------------------------------------------------

        # foot IK

        # "z-x",
        t_align = transform.getTransformLookingAt(
            self.guide.apos[3], self.guide.apos[4], self.normal, "zx", False
        )

        if self.settings["ikOri"]:
            t = transform.getTransformFromPos(self.guide.pos["foot"])
            # t = transform.getTransformLookingAt(self.guide.pos["foot"],
            #                                     self.guide.pos["eff"],
            #                                     self.x_axis,
            #                                     "zx",
            #                                     False)
        else:
            t = t_align

        self.ik_cns = primitive.addTransform(
            self.root_ctl, self.getName("ik_cns"), t
        )

        self.ikcns_ctl = self.addCtl(
            self.ik_cns,
            "ikcns_ctl",
            t,
            self.color_ik,
            "null",
            w=self.size * 0.12,
            tp=self.ankle_ctl,
        )

        attribute.setInvertMirror(self.ikcns_ctl, ["tx"])
        attribute.lockAttribute(self.ikcns_ctl, ["sx", "sy", "sz", "v"])

        self.ik_ctl = self.addCtl(
            self.ikcns_ctl,
            "ik_ctl",
            t,
            self.color_ik,
            "cube",
            w=self.size * 0.12,
            h=self.size * 0.12,
            d=self.size * 0.12,
            tp=self.ikcns_ctl,
        )
        attribute.setKeyableAttributes(self.ik_ctl)
        attribute.setRotOrder(self.ik_ctl, "XZY")
        attribute.setInvertMirror(self.ik_ctl, ["tx", "ry", "rz"])
        attribute.lockAttribute(self.ik_ctl, ["sx", "sy", "sz", "v"])

        # 2 bones ik layer
        self.ik2b_ikCtl_ref = primitive.addTransform(
            self.ik_ctl, self.getName("ik2B_A_ref"), t_align
        )
        self.ik2b_bone_ref = primitive.addTransform(
            self.chain3bones[3], self.getName("ik2B_B_ref"), t_align
        )
        self.ik2b_blend = primitive.addTransform(
            self.ik_ctl, self.getName("ik2B_blend"), t_align
        )

        self.roll_ctl = self.addCtl(
            self.ik2b_blend,
            "roll_ctl",
            t_align,
            self.color_ik,
            "crossarrow",
            w=self.length2 * 0.5 * self.n_factor,
            tp=self.ik_ctl,
        )

        self.ik2b_ik_npo = primitive.addTransform(
            self.roll_ctl,
            self.getName("ik2B_ik_npo"),
            transform.getTransform(self.chain3bones[-1]),
        )

        self.ik2b_ik_ref = primitive.addTransformFromPos(
            self.ik2b_ik_npo,
            self.getName("ik2B_ik_ref"),
            self.guide.pos["ankle"],
        )

        attribute.lockAttribute(
            self.roll_ctl, ["tx", "ty", "tz", "sx", "sy", "sz", "v"]
        )

        # upv
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size * 0.5
        v += self.guide.apos[1]

        self.upv_lvl = primitive.addTransformFromPos(
            self.root, self.getName("upv_lvl"), v
        )
        self.upv_cns = primitive.addTransformFromPos(
            self.upv_lvl, self.getName("upv_cns"), v
        )

        self.upv_ctl = self.addCtl(
            self.upv_cns,
            "upv_ctl",
            transform.getTransform(self.upv_cns),
            self.color_ik,
            "diamond",
            w=self.size * 0.12,
            tp=self.ik_ctl,
        )

        attribute.setInvertMirror(self.upv_ctl, ["tx"])
        attribute.setKeyableAttributes(self.upv_ctl, ["tx", "ty", "tz"])

        # Soft IK objects 3 bones chain --------------------------------
        t = transform.getTransformLookingAt(
            self.guide.pos["root"],
            self.guide.pos["foot"],
            self.x_axis,
            "zx",
            False,
        )

        self.aim_tra = primitive.addTransform(
            self.root_ctl, self.getName("aimSoftIK"), t
        )

        t = transform.getTransformFromPos(self.guide.pos["foot"])
        self.wristSoftIK = primitive.addTransform(
            self.aim_tra, self.getName("wristSoftIK"), t
        )

        self.softblendLoc = primitive.addTransform(
            self.root, self.getName("softblendLoc"), t
        )

        # Soft IK objects 2 Bones chain ----------------------------
        t = transform.getTransformLookingAt(
            self.guide.pos["root"],
            self.guide.pos["ankle"],
            self.x_axis,
            "zx",
            False,
        )

        self.aim_tra2 = primitive.addTransform(
            self.root_ctl, self.getName("aimSoftIK2"), t
        )

        t = transform.getTransformFromPos(self.guide.pos["ankle"])

        self.ankleSoftIK = primitive.addTransform(
            self.aim_tra2, self.getName("ankleSoftIK"), t
        )

        self.softblendLoc2 = primitive.addTransform(
            self.root, self.getName("softblendLoc2"), t
        )

        # References --------------------------------------
        self.ik_ref = primitive.addTransform(
            self.ik_ctl,
            self.getName("ik_ref"),
            transform.getTransform(self.ik_ctl),
        )

        self.fk_ref = primitive.addTransform(
            self.fk_ctl[3],
            self.getName("fk_ref"),
            transform.getTransform(self.ik_ctl),
        )

        # twist references --------------------------------------
        self.rollRef = primitive.add2DChain(
            self.root,
            self.getName("rollChain"),
            self.guide.apos[:2],
            self.normal,
            False,
            self.WIP,
        )

        self.tws0_loc = primitive.addTransform(
            self.rollRef[0],
            self.getName("tws0_loc"),
            transform.getTransform(self.legBones[0]),
        )

        self.tws0_rot = primitive.addTransform(
            self.tws0_loc,
            self.getName("tws0_rot"),
            transform.getTransform(self.legBones[0]),
        )

        self.tws0_rot.setAttr("sx", 0.001)

        self.tws1_loc = primitive.addTransform(
            self.mid1_jnt,
            self.getName("tws1_loc"),
            transform.getTransform(self.mid1_jnt),
        )

        self.tws1_rot = primitive.addTransform(
            self.tws1_loc,
            self.getName("tws1_rot"),
            transform.getTransform(self.mid1_jnt),
        )

        self.tws1_rot.setAttr("sx", 0.001)

        self.tws2_loc = primitive.addTransform(
            self.mid2_jnt,
            self.getName("tws2_loc"),
            transform.getTransform(self.mid2_jnt),
        )

        self.tws2_rot = primitive.addTransform(
            self.tws2_loc,
            self.getName("tws2_rot"),
            transform.getTransform(self.mid2_jnt),
        )

        self.tws2_rot.setAttr("sx", 0.001)

        self.tws3_loc = primitive.addTransform(
            self.legBones[3],
            self.getName("tws3_loc"),
            transform.getTransform(self.legBones[3]),
        )

        self.tws3_rot = primitive.addTransform(
            self.tws3_loc,
            self.getName("tws3_rot"),
            transform.getTransform(self.legBones[3]),
        )
        self.tws3_rot.setAttr("sx", 0.001)

        self.tws3_drv = primitive.addTransform(
            self.legBones[2],
            self.getName("tws3_drv"),
            transform.getTransform(self.legBones[3]),
        )

        self.tws3_drv.setAttr("sx", 0.001)

        # ---------------------------------------------------------------
        # Free tangent twist system (ported from leg_2jnt_freeTangents_01)
        # ---------------------------------------------------------------
        # The reference has a single junction (mid_ctl / knee) between 2
        # twist chains (uplegTwistChain, lowlegTwistChain). Here we have 2
        # junctions (knee_ctl, ankle_ctl) between 3 twist chains
        # (uplegTwistChain, midlegTwistChain, lowlegTwistChain). The
        # reference's tws1A/tws1B (near-knee stable-orientation parents,
        # living under mid_ctl) and kneeTangent_ctl (raw, parented directly
        # under mid_ctl) pattern is mirrored at both junctions.

        # Roll twist chains -----------------------------------------
        # upleg: root -> knee
        self.uplegChainPos = []
        ii = 1.0 / (self.settings["div0"] + 1)
        i = 0.0
        for p in range(self.settings["div0"] + 2):
            self.uplegChainPos.append(
                vector.linearlyInterpolate(
                    self.guide.pos["root"], self.guide.pos["knee"], blend=i
                )
            )
            i = i + ii

        self.uplegTwistChain = primitive.add2DChain(
            self.root,
            self.getName("uplegTwist%s_jnt"),
            self.uplegChainPos,
            self.normal,
            False,
            self.WIP,
        )

        # midleg: knee -> ankle
        self.midlegChainPos = []
        ii = 1.0 / (self.settings["div1"] + 1)
        i = 0.0
        for p in range(self.settings["div1"] + 2):
            self.midlegChainPos.append(
                vector.linearlyInterpolate(
                    self.guide.pos["knee"], self.guide.pos["ankle"], blend=i
                )
            )
            i = i + ii

        self.midlegTwistChain = primitive.add2DChain(
            self.root,
            self.getName("midlegTwist%s_jnt"),
            self.midlegChainPos,
            self.normal,
            False,
            self.WIP,
        )
        pm.parent(self.midlegTwistChain[0], self.knee_ctl)

        # lowleg: ankle -> foot
        self.lowlegChainPos = []
        ii = 1.0 / (self.settings["div2"] + 1)
        i = 0.0
        for p in range(self.settings["div2"] + 2):
            self.lowlegChainPos.append(
                vector.linearlyInterpolate(
                    self.guide.pos["ankle"], self.guide.pos["foot"], blend=i
                )
            )
            i = i + ii

        self.lowlegTwistChain = primitive.add2DChain(
            self.root,
            self.getName("lowlegTwist%s_jnt"),
            self.lowlegChainPos,
            self.normal,
            False,
            self.WIP,
        )
        pm.parent(self.lowlegTwistChain[0], self.ankle_ctl)

        # Foot Aux chain and nonroll (mirrors reference's hand aux chain)
        self.auxChainPos = []
        ii = 0.5
        i = 0.0
        for p in range(3):
            self.auxChainPos.append(
                vector.linearlyInterpolate(
                    self.guide.pos["foot"], self.guide.pos["eff"], blend=i
                )
            )
            i = i + ii
        t = self.root.getMatrix(worldSpace=True)

        self.aux_npo = primitive.addTransform(
            self.root, self.getName("aux_npo"), t
        )
        self.auxTwistChain = primitive.add2DChain(
            self.aux_npo,
            self.getName("auxTwist%s_jnt"),
            self.auxChainPos[:3],
            self.normal,
            False,
            self.WIP,
        )

        # Non Roll join ref ---------------------------------
        self.uplegRollRef = primitive.add2DChain(
            self.root,
            self.getName("uplegRollRef%s_jnt"),
            self.uplegChainPos[:2],
            self.normal,
            False,
            self.WIP,
        )

        self.lowlegRollRef = primitive.add2DChain(
            self.aux_npo,
            self.getName("lowlegRollRef%s_jnt"),
            self.auxChainPos[:2],
            self.normal,
            False,
            self.WIP,
        )

        # tws_ref (equivalent of the reference's ankle-side tws_ref, but at
        # the foot end for the aux/nonroll chain)
        t = transform.getRotationFromAxis(
            datatypes.Vector(0, -1, 0), self.normal, "xz", self.negate
        )
        t = transform.setMatrixPosition(t, self.guide.pos["foot"])
        self.tws_ref = primitive.addTransform(
            self.legBones[3], self.getName("tws_ref"), t
        )

        # Twist junction orientation locators ------------------------
        # tws1A/tws1B live under knee_ctl (like reference's under mid_ctl)
        tA_knee = transform.getTransformLookingAt(
            self.guide.apos[0],
            self.guide.apos[1],
            self.normal,
            "xz",
            self.negate,
        )
        tA_knee = transform.setMatrixPosition(tA_knee, self.guide.apos[1])
        tB_knee = transform.getTransformLookingAt(
            self.guide.apos[1],
            self.guide.apos[2],
            self.normal,
            "xz",
            self.negate,
        )

        self.tws1A_npo = primitive.addTransform(
            self.knee_ctl, self.getName("tws1A_npo"), tA_knee
        )
        self.tws1A_loc = primitive.addTransform(
            self.tws1A_npo, self.getName("tws1A_loc"), tA_knee
        )
        self.tws1B_npo = primitive.addTransform(
            self.knee_ctl, self.getName("tws1B_npo"), tB_knee
        )
        self.tws1B_loc = primitive.addTransform(
            self.tws1B_npo, self.getName("tws1B_loc"), tB_knee
        )

        # tws2A/tws2B live under ankle_ctl
        tA_ankle = transform.getTransformLookingAt(
            self.guide.apos[1],
            self.guide.apos[2],
            self.normal,
            "xz",
            self.negate,
        )
        tA_ankle = transform.setMatrixPosition(tA_ankle, self.guide.apos[2])
        tB_ankle = transform.getTransformLookingAt(
            self.guide.apos[2],
            self.guide.apos[3],
            self.normal,
            "xz",
            self.negate,
        )

        self.tws2A_npo = primitive.addTransform(
            self.ankle_ctl, self.getName("tws2A_npo"), tA_ankle
        )
        self.tws2A_loc = primitive.addTransform(
            self.tws2A_npo, self.getName("tws2A_loc"), tA_ankle
        )
        self.tws2B_npo = primitive.addTransform(
            self.ankle_ctl, self.getName("tws2B_npo"), tB_ankle
        )
        self.tws2B_loc = primitive.addTransform(
            self.tws2B_npo, self.getName("tws2B_loc"), tB_ankle
        )

        # Divisions ----------------------------------------
        # We have at least one division at the start, the end, one for the
        # knee and one for the ankle (2 junctions -> +5, one more than the
        # 2jnt reference's +4, which only has 1 junction).
        o_set = self.settings
        self.divisions = o_set["div0"] + o_set["div1"] + o_set["div2"] + 5

        self.div_cns = []
        for i in range(self.divisions):
            div_cns = primitive.addTransform(
                self.root_ctl, self.getName("div%s_loc" % i)
            )
            self.div_cns.append(div_cns)
            self.jnt_pos.append([div_cns, i])

        # End reference ------------------------------------
        # To help the deformation on the foot
        self.end_ref = primitive.addTransform(
            self.tws3_rot,
            self.getName("end_ref"),
            transform.getTransform(self.legBones[3]),
        )

        for a in "xyz":
            self.end_ref.attr("s%s" % a).set(1.0)
        if self.negate:
            self.end_ref.attr("ry").set(-180.0)

        t = transform.getTransform(self.end_ref)
        pos = self.guide.pos["ankle"]
        pos[1] = 0
        self.squash_npo = primitive.addTransformFromPos(
            self.end_ref, self.getName("squashEnd_npo"), pos
        )
        self.squash_scl = primitive.addTransformFromPos(
            self.squash_npo, self.getName("squashEnd_scl"), pos
        )
        self.squash_ref = primitive.addTransform(
            self.squash_scl,
            self.getName("squash_ref"),
            transform.getTransform(self.legBones[3]),
        )

        self.jnt_pos.append([self.squash_ref, "end"])

        # Tangent controls -------------------------------------------
        # upleg segment (root -> knee), mirrors reference uplegTangentA/B
        t = transform.getInterpolateTransformMatrix(
            self.fk_ctl[0], self.tws1A_npo, 0.5
        )
        self.uplegTangentA_loc = primitive.addTransform(
            self.root_ctl,
            self.getName("uplegTangentA_loc"),
            self.fk_ctl[0].getMatrix(worldSpace=True),
        )

        self.uplegTangentA_npo = primitive.addTransform(
            self.uplegTangentA_loc, self.getName("uplegTangentA_npo"), t
        )

        self.uplegTangentA_ctl = self.addCtl(
            self.uplegTangentA_npo,
            "uplegTangentA_ctl",
            t,
            self.color_ik,
            "circle",
            w=self.size * 0.2,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.knee_ctl,
        )

        if self.negate:
            self.uplegTangentA_npo.rz.set(180)
            self.uplegTangentA_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.uplegTangentA_ctl, self.t_params)

        t = transform.getInterpolateTransformMatrix(
            self.fk_ctl[0], self.tws1A_npo, 0.9
        )
        self.uplegTangentB_npo = primitive.addTransform(
            self.tws1A_loc, self.getName("uplegTangentB_npo"), t
        )

        self.uplegTangentB_ctl = self.addCtl(
            self.uplegTangentB_npo,
            "uplegTangentB_ctl",
            t,
            self.color_ik,
            "circle",
            w=self.size * 0.1,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.knee_ctl,
        )

        if self.negate:
            self.uplegTangentB_npo.rz.set(180)
            self.uplegTangentB_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.uplegTangentB_ctl, self.t_params)

        # Knee junction tangent - raw, parented directly under knee_ctl,
        # exactly matching the reference's kneeTangent_ctl under mid_ctl.
        t = self.knee_ctl.getMatrix(worldSpace=True)
        self.kneeTangent_npo = primitive.addTransform(
            self.knee_ctl, self.getName("kneeTangent_npo"), t
        )

        self.kneeTangent_ctl = self.addCtl(
            self.kneeTangent_npo,
            "kneeTangent_ctl",
            t,
            self.color_fk,
            "circle",
            w=self.size * 0.25,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.knee_ctl,
        )

        if self.negate:
            self.kneeTangent_npo.rz.set(180)
            self.kneeTangent_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.kneeTangent_ctl, self.t_params)

        # midleg segment (knee -> ankle), mirrors reference's lowlegTangentA
        # (near junction) and adds a mirrored "B" side near the ankle
        # junction, following the same tws1B/tws2A stable-parent pattern.
        tC_mid = self.tws2A_npo.getMatrix(worldSpace=True)
        tC_mid = transform.setMatrixPosition(tC_mid, self.guide.apos[2])
        t = transform.getInterpolateTransformMatrix(self.tws1B_npo, tC_mid, 0.1)
        self.midlegTangentA_npo = primitive.addTransform(
            self.tws1B_loc, self.getName("midlegTangentA_npo"), t
        )

        self.midlegTangentA_ctl = self.addCtl(
            self.midlegTangentA_npo,
            "midlegTangentA_ctl",
            t,
            self.color_ik,
            "circle",
            w=self.size * 0.1,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.knee_ctl,
        )

        if self.negate:
            self.midlegTangentA_npo.rz.set(180)
            self.midlegTangentA_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.midlegTangentA_ctl, self.t_params)

        t = transform.getInterpolateTransformMatrix(self.tws1B_npo, tC_mid, 0.9)
        self.midlegTangentB_npo = primitive.addTransform(
            self.tws2A_loc, self.getName("midlegTangentB_npo"), t
        )

        self.midlegTangentB_ctl = self.addCtl(
            self.midlegTangentB_npo,
            "midlegTangentB_ctl",
            t,
            self.color_ik,
            "circle",
            w=self.size * 0.1,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.ankle_ctl,
        )

        if self.negate:
            self.midlegTangentB_npo.rz.set(180)
            self.midlegTangentB_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.midlegTangentB_ctl, self.t_params)

        # Ankle junction tangent - raw, parented directly under ankle_ctl,
        # matching the same known-good pattern as kneeTangent_ctl.
        t = self.ankle_ctl.getMatrix(worldSpace=True)
        self.ankleTangent_npo = primitive.addTransform(
            self.ankle_ctl, self.getName("ankleTangent_npo"), t
        )

        self.ankleTangent_ctl = self.addCtl(
            self.ankleTangent_npo,
            "ankleTangent_ctl",
            t,
            self.color_fk,
            "circle",
            w=self.size * 0.25,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.ankle_ctl,
        )

        if self.negate:
            self.ankleTangent_npo.rz.set(180)
            self.ankleTangent_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.ankleTangent_ctl, self.t_params)

        # lowleg segment (ankle -> foot), mirrors reference's
        # lowlegTangentA (near junction) / lowlegTangentB (near foot end)
        tC_low = self.tws2B_npo.getMatrix(worldSpace=True)
        tC_low = transform.setMatrixPosition(tC_low, self.guide.apos[3])
        t = transform.getInterpolateTransformMatrix(self.tws2B_npo, tC_low, 0.1)
        self.lowlegTangentA_npo = primitive.addTransform(
            self.tws2B_loc, self.getName("lowlegTangentA_npo"), t
        )

        self.lowlegTangentA_ctl = self.addCtl(
            self.lowlegTangentA_npo,
            "lowlegTangentA_ctl",
            t,
            self.color_ik,
            "circle",
            w=self.size * 0.1,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.ankle_ctl,
        )

        if self.negate:
            self.lowlegTangentA_npo.rz.set(180)
            self.lowlegTangentA_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.lowlegTangentA_ctl, self.t_params)

        t = transform.getInterpolateTransformMatrix(self.tws2B_npo, tC_low, 0.5)

        self.lowlegTangentB_loc = primitive.addTransform(
            self.root, self.getName("lowlegTangentB_loc"), tC_low
        )

        self.lowlegTangentB_npo = primitive.addTransform(
            self.lowlegTangentB_loc, self.getName("lowlegTangentB_npo"), t
        )

        self.lowlegTangentB_ctl = self.addCtl(
            self.lowlegTangentB_npo,
            "lowlegTangentB_ctl",
            t,
            self.color_ik,
            "circle",
            w=self.size * 0.2,
            ro=datatypes.Vector(0, 0, 1.570796),
            tp=self.ankle_ctl,
        )

        if self.negate:
            self.lowlegTangentB_npo.rz.set(180)
            self.lowlegTangentB_npo.sz.set(-1)
        attribute.setKeyableAttributes(self.lowlegTangentB_ctl, self.t_params)

        # match IK FK references
        self.match_fk0_off = self.add_match_ref(
            self.fk_ctl[1], self.root, "matchFk0_npo", False
        )

        self.match_fk0 = self.add_match_ref(
            self.fk_ctl[0], self.match_fk0_off, "fk0_mth"
        )

        self.match_fk1_off = self.add_match_ref(
            self.fk_ctl[2], self.root, "matchFk1_npo", False
        )

        self.match_fk1 = self.add_match_ref(
            self.fk_ctl[1], self.match_fk1_off, "fk1_mth"
        )

        self.match_fk2_off = self.add_match_ref(
            self.fk_ctl[3], self.root, "matchFk2_npo", False
        )

        self.match_fk2 = self.add_match_ref(
            self.fk_ctl[2], self.match_fk2_off, "fk2_mth"
        )

        self.match_fk3 = self.add_match_ref(
            self.fk_ctl[3], self.legBonesIK[-1], "fk3_mth"
        )

        self.match_ik = self.add_match_ref(self.ik_ctl, self.fk3_ctl, "ik_mth")
        self.match_roll = self.add_match_ref(self.roll_ctl, self.fk2_ctl, "roll_mth")

        self.match_ikUpv = self.add_match_ref(
            self.upv_ctl, self.fk0_ctl, "upv_mth"
        )

        # add visual reference
        self.line_ref = icon.connection_display_curve(
            self.getName("visalRef"), [self.upv_ctl, self.knee_ctl]
        )

    def addAttributes(self):

        self.blend_att = self.addAnimParam(
            "blend", "Fk/Ik Blend", "double", self.settings["blend"], 0, 1
        )
        self.fullIK_attr = self.addAnimParam(
            "fullIK",
            "Full 3 bones IK",
            "double",
            self.settings["full3BonesIK"],
            0,
            1,
        )
        self.soft_attr = self.addAnimParam(
            "softIKRange", "Soft IK Range", "double", 0.0001, 0.0001, 100
        )
        self.softSpeed_attr = self.addAnimParam(
            "softIKSpeed", "Soft IK Speed", "double", 2.5, 1.001, 10
        )
        self.stretch_attr = self.addAnimParam(
            "stretch", "Stretch", "double", 0, 0, 1
        )
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )
        self.roll_att = self.addAnimParam(
            "roll", "Roll", "double", 0, -180, 180
        )
        self.roundness_att = self.addAnimParam(
            "roundness", "Roundness", "double", 0, 0, 1
        )
        self.tangentVis_att = self.addAnimParam(
            "Tangent_vis", "Tangent vis", "bool", False
        )

        self.boneALenghtMult_attr = self.addAnimParam(
            "boneALenMult", "Bone A Mult", "double", 1
        )
        self.boneBLenghtMult_attr = self.addAnimParam(
            "boneBLenMult", "Bone B Mult", "double", 1
        )
        self.boneCLenghtMult_attr = self.addAnimParam(
            "boneCLenMult", "Bone C Mult", "double", 1
        )
        self.boneALenght_attr = self.addAnimParam(
            "boneALen", "Bone A Length", "double", self.length0, keyable=False
        )
        self.boneBLenght_attr = self.addAnimParam(
            "boneBLen", "Bone B Length", "double", self.length1, keyable=False
        )
        self.boneCLenght_attr = self.addAnimParam(
            "boneCLen", "Bone C Length", "double", self.length2, keyable=False
        )

        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["ikrefarray"].split(",")
            )
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam(
                    "ikref", "Ik Ref", 0, ref_names
                )

        if self.settings["upvrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["upvrefarray"].split(",")
            )
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.upvref_att = self.addAnimEnumParam(
                    "upvref", "UpV Ref", 0, ref_names
                )

        if self.settings["kneerefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["kneerefarray"].split(",")
            )
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.kneeref_att = self.addAnimEnumParam(
                    "kneeref", "Knee Ref", 0, ref_names
                )

        if self.settings["anklerefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["anklerefarray"].split(",")
            )
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.ankleref_att = self.addAnimEnumParam(
                    "ankleref", "Ankle Ref", 0, ref_names
                )
        if self.validProxyChannels:
            attribute.addProxyAttribute(
                [self.blend_att, self.roundness_att],
                [
                    self.fk0_ctl,
                    self.fk1_ctl,
                    self.fk2_ctl,
                    self.ik_ctl,
                    self.upv_ctl,
                ],
            )
            attribute.addProxyAttribute(
                self.roll_att, [self.ik_ctl, self.upv_ctl]
            )

        self.foot_squash_att = self.addAnimParam(
            "footSquash", "Foot Squash", "double", 1, 0.01, 1.9
        )

        # Setup ------------------------------------------
        # Eval Fcurve
        if self.guide.paramDefs["st_profile"].value:
            self.st_value = self.guide.paramDefs["st_profile"].value
            self.sq_value = self.guide.paramDefs["sq_profile"].value
        else:
            self.st_value = fcurve.getFCurveValues(
                self.settings["st_profile"], self.divisions
            )
            self.sq_value = fcurve.getFCurveValues(
                self.settings["sq_profile"], self.divisions
            )

        self.st_att = [
            self.addSetupParam(
                "stretch_%s" % i,
                "Stretch %s" % i,
                "double",
                self.st_value[i],
                -1,
                0,
            )
            for i in range(self.divisions)
        ]

        self.sq_att = [
            self.addSetupParam(
                "squash_%s" % i,
                "Squash %s" % i,
                "double",
                self.sq_value[i],
                0,
                1,
            )
            for i in range(self.divisions)
        ]

        self.resample_att = self.addSetupParam(
            "resample", "Resample", "bool", True
        )
        self.absolute_att = self.addSetupParam(
            "absolute", "Absolute", "bool", False
        )

        defValu = self.chain3bones[1].attr("jointOrientZ").get() / 2
        self.kneeFlipOffset_att = self.addSetupParam(
            "kneeFlipOffset", "Knee Flip Offset", "double", defValu, -180, 180
        )
        defValu = self.chain3bones[2].attr("jointOrientZ").get() / 2
        self.ankleFlipOffset_att = self.addSetupParam(
            "ankleFlipOffset",
            "Ankle Flip Offset",
            "double",
            defValu,
            -180,
            180,
        )

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        """Create operators and set the relations for the component rig

        Apply operators, constraints, expressions to the hierarchy.
        In order to keep the code clean and easier to debug,
        we shouldn't create any new object in this method.

        """
        # Soft condition
        soft_cond_node = node.createConditionNode(
            self.soft_attr, 0.0001, 4, 0.0001, self.soft_attr
        )
        self.soft_attr_cond = soft_cond_node.outColorR

        if self.settings["ikSolver"]:
            self.ikSolver = "ikRPsolver"
        else:
            pm.mel.eval("ikSpringSolver;")
            self.ikSolver = "ikSpringSolver"

        # 1 bone chain Upv ref ===============================
        self.ikHandleUpvRef = primitive.addIkHandle(
            self.root,
            self.getName("ikHandleLegChainUpvRef"),
            self.legChainUpvRef,
            "ikSCsolver",
        )
        pm.pointConstraint(self.ik_ctl, self.ikHandleUpvRef)
        # pm.parentConstraint(self.legChainUpvRef[0], self.upv_cns, mo=True)
        self.relatives_map_upv = {
            "Auto": self.legChainUpvRef[0],
        }

        # mid joints ================================================
        for xjnt, midJ in zip(
            self.legBones[1:3], [self.mid1_jnt, self.mid2_jnt]
        ):
            node.createPairBlend(None, xjnt, 0.5, 1, midJ)
            pm.connectAttr(xjnt + ".translate", midJ + ".translate", f=True)

        pm.parentConstraint(self.mid1_jnt, self.knee_lvl)
        pm.parentConstraint(self.mid2_jnt, self.ankle_lvl)

        # joint length multiply
        multJnt1_node = node.createMulNode(
            self.boneALenght_attr, self.boneALenghtMult_attr
        )
        multJnt2_node = node.createMulNode(
            self.boneBLenght_attr, self.boneBLenghtMult_attr
        )
        multJnt3_node = node.createMulNode(
            self.boneCLenght_attr, self.boneCLenghtMult_attr
        )

        # # IK 3 bones ===============================================

        self.ikHandle = primitive.addIkHandle(
            self.softblendLoc,
            self.getName("ik3BonesHandle"),
            self.chain3bones,
            "ikRPsolver",
            self.upv_ctl,
        )
        # we connect the ik spring solver after tu avoid flip issue that may
        # happend with the IKSprin solver
        if self.ikSolver == "ikSpringSolver":
            pm.connectAttr(
                "ikSpringSolver.message", self.ikHandle.ikSolver, force=True
            )

        # TwistTest
        if [
            round(elem, 4)
            for elem in transform.getTranslation(self.chain3bones[1])
        ] != [round(elem, 4) for elem in self.guide.apos[1]]:
            add_nodeTwist = node.createAddNode(180.0, self.roll_att)
        else:
            add_nodeTwist = node.createAddNode(0, self.roll_att)
        if self.negate:
            mulVal = 1
        else:
            mulVal = -1
        node.createMulNode(
            add_nodeTwist + ".output", mulVal, self.ikHandle.attr("twist")
        )

        # stable spring solver doble rotation
        pm.pointConstraint(self.root_ctl, self.chain3bones[0])

        # softIK 3 bones operators
        applyop.aimCns(
            self.aim_tra,
            self.ik_ref,
            axis="zx",
            wupType=4,
            wupVector=[1, 0, 0],
            wupObject=self.root_ctl,
            maintainOffset=False,
        )

        plusTotalLength_node = node.createPlusMinusAverage1D(
            [
                multJnt1_node.attr("outputX"),
                multJnt2_node.attr("outputX"),
                multJnt3_node.attr("outputX"),
            ]
        )

        subtract1_node = node.createPlusMinusAverage1D(
            [plusTotalLength_node.attr("output1D"), self.soft_attr_cond], 2
        )

        distance1_node = node.createDistNode(self.ik_ref, self.aim_tra)
        div1_node = node.createDivNode(1.0, self.rig.global_ctl + ".sx")
        mult1_node = node.createMulNode(
            distance1_node + ".distance", div1_node + ".outputX"
        )
        subtract2_node = node.createPlusMinusAverage1D(
            [mult1_node.attr("outputX"), subtract1_node.attr("output1D")], 2
        )
        div2_node = node.createDivNode(
            subtract2_node + ".output1D", self.soft_attr_cond
        )
        mult2_node = node.createMulNode(-1, div2_node + ".outputX")
        power_node = node.createPowNode(
            self.softSpeed_attr, mult2_node + ".outputX"
        )
        mult3_node = node.createMulNode(
            self.soft_attr_cond, power_node + ".outputX"
        )
        subtract3_node = node.createPlusMinusAverage1D(
            [
                plusTotalLength_node.attr("output1D"),
                mult3_node.attr("outputX"),
            ],
            2,
        )

        cond1_node = node.createConditionNode(
            self.soft_attr_cond,
            0,
            2,
            subtract3_node + ".output1D",
            plusTotalLength_node + ".output1D",
        )

        cond2_node = node.createConditionNode(
            mult1_node + ".outputX",
            subtract1_node + ".output1D",
            2,
            cond1_node + ".outColorR",
            mult1_node + ".outputX",
        )

        pm.connectAttr(cond2_node + ".outColorR", self.wristSoftIK + ".tz")

        # soft blend
        pc_node = pm.pointConstraint(
            self.wristSoftIK, self.ik_ref, self.softblendLoc
        )
        node.createReverseNode(
            self.stretch_attr, pc_node + ".target[0].targetWeight"
        )
        pm.connectAttr(
            self.stretch_attr, pc_node + ".target[1].targetWeight", f=True
        )

        # Stretch
        distance2_node = node.createDistNode(
            self.softblendLoc, self.wristSoftIK
        )
        mult4_node = node.createMulNode(
            distance2_node + ".distance", div1_node + ".outputX"
        )

        # bones
        for i, mulNode in enumerate(
            [multJnt1_node, multJnt2_node, multJnt3_node]
        ):

            div3_node = node.createDivNode(
                mulNode + ".outputX", plusTotalLength_node + ".output1D"
            )

            mult5_node = node.createMulNode(
                mult4_node + ".outputX", div3_node + ".outputX"
            )

            mult6_node = node.createMulNode(
                self.stretch_attr, mult5_node + ".outputX"
            )

            node.createPlusMinusAverage1D(
                [mulNode.attr("outputX"), mult6_node.attr("outputX")],
                1,
                self.chain3bones[i + 1] + ".tx",
            )

        # fix the squash Stretch when Full3bonesIK is 0
        pm.pointConstraint(self.legBones[3], self.tws3_drv, mo=True)

        # IK 2 bones ===============================================

        self.ikHandle2 = primitive.addIkHandle(
            self.softblendLoc2,
            self.getName("ik2BonesHandle"),
            self.chain2bones,
            "ikRPsolver",
            self.upv_ctl,
        )

        node.createMulNode(self.roll_att, mulVal, self.ikHandle2.attr("twist"))

        # stable spring solver doble rotation
        pm.pointConstraint(self.root_ctl, self.chain2bones[0])

        parentc_node = pm.parentConstraint(
            self.ik2b_ikCtl_ref, self.ik2b_bone_ref, self.ik2b_blend
        )

        node.createReverseNode(
            self.fullIK_attr, parentc_node + ".target[0].targetWeight"
        )

        pm.connectAttr(
            self.fullIK_attr, parentc_node + ".target[1].targetWeight", f=True
        )

        # softIK 2 bones operators
        applyop.aimCns(
            self.aim_tra2,
            self.ik2b_ik_ref,
            axis="zx",
            wupType=4,
            wupVector=[1, 0, 0],
            wupObject=self.root_ctl,
            maintainOffset=False,
        )

        plusTotalLength_node = node.createPlusMinusAverage1D(
            [multJnt1_node.attr("outputX"), multJnt2_node.attr("outputX")]
        )

        subtract1_node = node.createPlusMinusAverage1D(
            [plusTotalLength_node.attr("output1D"), self.soft_attr_cond], 2
        )

        distance1_node = node.createDistNode(self.ik2b_ik_ref, self.aim_tra2)
        div1_node = node.createDivNode(1, self.rig.global_ctl + ".sx")

        mult1_node = node.createMulNode(
            distance1_node + ".distance", div1_node + ".outputX"
        )

        subtract2_node = node.createPlusMinusAverage1D(
            [mult1_node.attr("outputX"), subtract1_node.attr("output1D")], 2
        )

        div2_node = node.createDivNode(
            subtract2_node + ".output1D", self.soft_attr_cond
        )

        mult2_node = node.createMulNode(-1, div2_node + ".outputX")

        power_node = node.createPowNode(
            self.softSpeed_attr, mult2_node + ".outputX"
        )

        mult3_node = node.createMulNode(
            self.soft_attr_cond, power_node + ".outputX"
        )

        subtract3_node = node.createPlusMinusAverage1D(
            [
                plusTotalLength_node.attr("output1D"),
                mult3_node.attr("outputX"),
            ],
            2,
        )

        cond1_node = node.createConditionNode(
            self.soft_attr_cond,
            0,
            2,
            subtract3_node + ".output1D",
            plusTotalLength_node + ".output1D",
        )

        cond2_node = node.createConditionNode(
            mult1_node + ".outputX",
            subtract1_node + ".output1D",
            2,
            cond1_node + ".outColorR",
            mult1_node + ".outputX",
        )

        pm.connectAttr(cond2_node + ".outColorR", self.ankleSoftIK + ".tz")

        # soft blend
        pc_node = pm.pointConstraint(
            self.ankleSoftIK, self.ik2b_ik_ref, self.softblendLoc2
        )
        node.createReverseNode(
            self.stretch_attr, pc_node + ".target[0].targetWeight"
        )
        pm.connectAttr(
            self.stretch_attr, pc_node + ".target[1].targetWeight", f=True
        )

        # Stretch
        distance2_node = node.createDistNode(
            self.softblendLoc2, self.ankleSoftIK
        )

        mult4_node = node.createMulNode(
            distance2_node + ".distance", div1_node + ".outputX"
        )

        for i, mulNode in enumerate([multJnt1_node, multJnt2_node]):
            div3_node = node.createDivNode(
                mulNode + ".outputX", plusTotalLength_node + ".output1D"
            )

            mult5_node = node.createMulNode(
                mult4_node + ".outputX", div3_node + ".outputX"
            )

            mult6_node = node.createMulNode(
                self.stretch_attr, mult5_node + ".outputX"
            )

            node.createPlusMinusAverage1D(
                [mulNode.attr("outputX"), mult6_node.attr("outputX")],
                1,
                self.chain2bones[i + 1] + ".tx",
            )

        # IK/FK connections

        for i, x in enumerate(self.fk_ctl):
            pm.parentConstraint(x, self.legBonesFK[i], mo=True)

        for i, x in enumerate([self.chain2bones[0], self.chain2bones[1]]):
            pm.parentConstraint(x, self.legBonesIK[i], mo=True)

        pm.pointConstraint(self.ik2b_ik_ref, self.legBonesIK[2])
        applyop.aimCns(
            self.legBonesIK[2],
            self.roll_ctl,
            axis="xy",
            wupType=4,
            wupVector=[0, 1, 0],
            wupObject=self.legBonesIK[1],
            maintainOffset=False,
        )

        pm.connectAttr(
            self.chain3bones[-1].attr("tx"), self.legBonesIK[-1].attr("tx")
        )
        # foot twist roll
        pm.orientConstraint(self.ik_ref, self.legBonesIK[-1], mo=True)

        node.createMulNode(
            -1, self.chain3bones[-1].attr("tx"), self.ik2b_ik_ref.attr("tx")
        )

        for i, x in enumerate(self.legBones):
            node.createPairBlend(
                self.legBonesFK[i], self.legBonesIK[i], self.blend_att, 1, x
            )

        # Twist references ----------------------------------------

        self.ikhArmRef, self.tmpCrv = applyop.splineIK(
            self.getName("legRollRef"),
            self.rollRef,
            parent=self.root,
            cParent=self.legBones[0],
        )

        # Drive the twist locators from the knee / ankle control WORLD
        # matrix (not their local channels). This way they follow the
        # control whether it is moved directly or pinned to a reference.
        initRound = 0.001
        self.tws1_rot.attr("sx").set(initRound)
        tws1_mm = applyop.gear_mulmatrix_op(
            self.knee_ctl.attr("worldMatrix[0]"),
            self.tws1_loc.attr("parentInverseMatrix[0]"),
        )
        tws1_dm = node.createDecomposeMatrixNode(tws1_mm + ".output")
        pm.connectAttr(tws1_dm + ".outputTranslate", self.tws1_loc.attr("t"))
        for x in "xy":
            pm.connectAttr(
                tws1_dm + ".outputRotate%s" % x.upper(),
                self.tws1_loc.attr("r" + x),
            )

        self.tws2_rot.attr("sx").set(initRound)
        tws2_mm = applyop.gear_mulmatrix_op(
            self.ankle_ctl.attr("worldMatrix[0]"),
            self.tws2_loc.attr("parentInverseMatrix[0]"),
        )
        tws2_dm = node.createDecomposeMatrixNode(tws2_mm + ".output")
        pm.connectAttr(tws2_dm + ".outputTranslate", self.tws2_loc.attr("t"))
        for x in "xy":
            pm.connectAttr(
                tws2_dm + ".outputRotate%s" % x.upper(),
                self.tws2_loc.attr("r" + x),
            )

        # Flip Offset ----------------------------------------
        pm.connectAttr(self.ankleFlipOffset_att, self.tws2_loc.attr("rz"))
        pm.connectAttr(self.kneeFlipOffset_att, self.tws1_loc.attr("rz"))

        # connect roll rotation driver reference
        pm.orientConstraint(
            self.legBones[3],
            self.tws3_drv,
            skip=["y", "z"],
            maintainOffset=True,
            weight=1,
        )

        # ---------------------------------------------------------------
        # Free tangent twist system operators
        # ---------------------------------------------------------------

        # spline IK for twist jnts
        self.ikhUpLegTwist, self.uplegTwistCrv = applyop.splineIK(
            self.getName("uplegTwist"),
            self.uplegTwistChain,
            parent=self.root,
            cParent=self.legBones[0],
        )

        self.ikhMidLegTwist, self.midlegTwistCrv = applyop.splineIK(
            self.getName("midlegTwist"),
            self.midlegTwistChain,
            parent=self.root,
            cParent=self.mid1_jnt,
        )

        self.ikhLowLegTwist, self.lowlegTwistCrv = applyop.splineIK(
            self.getName("lowlegTwist"),
            self.lowlegTwistChain,
            parent=self.root,
            cParent=self.mid2_jnt,
        )

        # references
        self.ikhUpLegRef, self.tmpCrvA = applyop.splineIK(
            self.getName("uplegRollRef"),
            self.uplegRollRef,
            parent=self.root,
            cParent=self.legBones[0],
        )

        self.ikhLowLegRef, self.tmpCrvB = applyop.splineIK(
            self.getName("lowlegRollRef"),
            self.lowlegRollRef,
            parent=self.root,
            cParent=self.legBones[3],
        )

        self.ikhAuxTwist, self.tmpCrvC = applyop.splineIK(
            self.getName("auxTwist"),
            self.auxTwistChain,
            parent=self.root,
            cParent=self.legBones[3],
        )

        # setting connexions for ikhUpLegTwist
        self.ikhUpLegTwist.attr("dTwistControlEnable").set(True)
        self.ikhUpLegTwist.attr("dWorldUpType").set(4)
        self.ikhUpLegTwist.attr("dWorldUpAxis").set(3)
        self.ikhUpLegTwist.attr("dWorldUpVectorZ").set(1.0)
        self.ikhUpLegTwist.attr("dWorldUpVectorY").set(0.0)
        self.ikhUpLegTwist.attr("dWorldUpVectorEndZ").set(1.0)
        self.ikhUpLegTwist.attr("dWorldUpVectorEndY").set(0.0)
        pm.connectAttr(
            self.uplegRollRef[0].attr("worldMatrix[0]"),
            self.ikhUpLegTwist.attr("dWorldUpMatrix"),
        )
        pm.connectAttr(
            self.legBones[0].attr("worldMatrix[0]"),
            self.ikhUpLegTwist.attr("dWorldUpMatrixEnd"),
        )

        # setting connexions for ikhAuxTwist
        self.ikhAuxTwist.attr("dTwistControlEnable").set(True)
        self.ikhAuxTwist.attr("dWorldUpType").set(4)
        self.ikhAuxTwist.attr("dWorldUpAxis").set(3)
        self.ikhAuxTwist.attr("dWorldUpVectorZ").set(1.0)
        self.ikhAuxTwist.attr("dWorldUpVectorY").set(0.0)
        self.ikhAuxTwist.attr("dWorldUpVectorEndZ").set(1.0)
        self.ikhAuxTwist.attr("dWorldUpVectorEndY").set(0.0)
        pm.connectAttr(
            self.lowlegRollRef[0].attr("worldMatrix[0]"),
            self.ikhAuxTwist.attr("dWorldUpMatrix"),
        )
        pm.connectAttr(
            self.tws_ref.attr("worldMatrix[0]"),
            self.ikhAuxTwist.attr("dWorldUpMatrixEnd"),
        )
        pm.connectAttr(
            self.auxTwistChain[1].attr("rx"), self.ikhLowLegTwist.attr("twist")
        )

        pm.parentConstraint(self.legBones[3], self.aux_npo, maintainOffset=True)

        # scale leg length for the twist chains (not the squash and stretch)
        arclen_node = pm.arclen(self.uplegTwistCrv, ch=True)
        alAttrUpLeg = arclen_node.attr("arcLength")
        muldiv_nodeUpLeg = pm.createNode("multiplyDivide")
        pm.connectAttr(
            arclen_node.attr("arcLength"), muldiv_nodeUpLeg.attr("input1X")
        )
        muldiv_nodeUpLeg.attr("input2X").set(alAttrUpLeg.get())
        muldiv_nodeUpLeg.attr("operation").set(2)
        for jnt in self.uplegTwistChain:
            pm.connectAttr(muldiv_nodeUpLeg.attr("outputX"), jnt.attr("sx"))

        arclen_node = pm.arclen(self.midlegTwistCrv, ch=True)
        alAttrMidLeg = arclen_node.attr("arcLength")
        muldiv_nodeMidLeg = pm.createNode("multiplyDivide")
        pm.connectAttr(
            arclen_node.attr("arcLength"), muldiv_nodeMidLeg.attr("input1X")
        )
        muldiv_nodeMidLeg.attr("input2X").set(alAttrMidLeg.get())
        muldiv_nodeMidLeg.attr("operation").set(2)
        for jnt in self.midlegTwistChain:
            pm.connectAttr(muldiv_nodeMidLeg.attr("outputX"), jnt.attr("sx"))

        arclen_node = pm.arclen(self.lowlegTwistCrv, ch=True)
        alAttrLowLeg = arclen_node.attr("arcLength")
        muldiv_nodeLowLeg = pm.createNode("multiplyDivide")
        pm.connectAttr(
            arclen_node.attr("arcLength"), muldiv_nodeLowLeg.attr("input1X")
        )
        muldiv_nodeLowLeg.attr("input2X").set(alAttrLowLeg.get())
        muldiv_nodeLowLeg.attr("operation").set(2)
        for jnt in self.lowlegTwistChain:
            pm.connectAttr(muldiv_nodeLowLeg.attr("outputX"), jnt.attr("sx"))

        # scale compensation for the first twist joint of each chain
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(
            self.root.attr("worldMatrix[0]"), dm_node.attr("inputMatrix")
        )
        pm.connectAttr(
            dm_node.attr("outputScale"),
            self.uplegTwistChain[0].attr("inverseScale"),
        )
        pm.connectAttr(
            dm_node.attr("outputScale"),
            self.midlegTwistChain[0].attr("inverseScale"),
        )
        pm.connectAttr(
            dm_node.attr("outputScale"),
            self.lowlegTwistChain[0].attr("inverseScale"),
        )

        # tangent controls (roundness bias) -------------------------
        # Knee junction (tws1A / tws1B)
        muldiv_node = pm.createNode("multiplyDivide")
        muldiv_node.attr("input2X").set(-1)
        pm.connectAttr(self.tws1A_npo.attr("rz"), muldiv_node.attr("input1X"))
        muldiv_nodeBias = pm.createNode("multiplyDivide")
        pm.connectAttr(
            muldiv_node.attr("outputX"), muldiv_nodeBias.attr("input1X")
        )
        pm.connectAttr(self.roundness_att, muldiv_nodeBias.attr("input2X"))
        pm.connectAttr(
            muldiv_nodeBias.attr("outputX"), self.tws1A_loc.attr("rz")
        )
        if self.negate:
            axis = "xz"
        else:
            axis = "-xz"
        applyop.aimCns(
            self.tws1A_npo,
            self.tws0_loc,
            axis=axis,
            wupType=2,
            wupVector=[0, 0, 1],
            wupObject=self.knee_ctl,
            maintainOffset=False,
        )

        muldiv_node = pm.createNode("multiplyDivide")
        muldiv_node.attr("input2X").set(-1)
        pm.connectAttr(self.tws1B_npo.attr("rz"), muldiv_node.attr("input1X"))
        muldiv_nodeBias = pm.createNode("multiplyDivide")
        pm.connectAttr(
            muldiv_node.attr("outputX"), muldiv_nodeBias.attr("input1X")
        )
        pm.connectAttr(self.roundness_att, muldiv_nodeBias.attr("input2X"))
        pm.connectAttr(
            muldiv_nodeBias.attr("outputX"), self.tws1B_loc.attr("rz")
        )
        if self.negate:
            axis = "-xz"
        else:
            axis = "xz"
        applyop.aimCns(
            self.tws1B_npo,
            self.tws2_loc,
            axis=axis,
            wupType=2,
            wupVector=[0, 0, 1],
            wupObject=self.knee_ctl,
            maintainOffset=False,
        )

        applyop.aimCns(
            self.uplegTangentA_loc,
            self.uplegTangentB_npo,
            axis=axis,
            wupType=2,
            wupVector=[0, 0, 1],
            wupObject=self.knee_ctl,
            maintainOffset=False,
        )

        # Ankle junction (tws2A / tws2B)
        muldiv_node = pm.createNode("multiplyDivide")
        muldiv_node.attr("input2X").set(-1)
        pm.connectAttr(self.tws2A_npo.attr("rz"), muldiv_node.attr("input1X"))
        muldiv_nodeBias = pm.createNode("multiplyDivide")
        pm.connectAttr(
            muldiv_node.attr("outputX"), muldiv_nodeBias.attr("input1X")
        )
        pm.connectAttr(self.roundness_att, muldiv_nodeBias.attr("input2X"))
        pm.connectAttr(
            muldiv_nodeBias.attr("outputX"), self.tws2A_loc.attr("rz")
        )
        if self.negate:
            axis = "xz"
        else:
            axis = "-xz"
        applyop.aimCns(
            self.tws2A_npo,
            self.tws1_loc,
            axis=axis,
            wupType=2,
            wupVector=[0, 0, 1],
            wupObject=self.ankle_ctl,
            maintainOffset=False,
        )

        muldiv_node = pm.createNode("multiplyDivide")
        muldiv_node.attr("input2X").set(-1)
        pm.connectAttr(self.tws2B_npo.attr("rz"), muldiv_node.attr("input1X"))
        muldiv_nodeBias = pm.createNode("multiplyDivide")
        pm.connectAttr(
            muldiv_node.attr("outputX"), muldiv_nodeBias.attr("input1X")
        )
        pm.connectAttr(self.roundness_att, muldiv_nodeBias.attr("input2X"))
        pm.connectAttr(
            muldiv_nodeBias.attr("outputX"), self.tws2B_loc.attr("rz")
        )
        if self.negate:
            axis = "-xz"
        else:
            axis = "xz"
        applyop.aimCns(
            self.tws2B_npo,
            self.tws3_loc,
            axis=axis,
            wupType=2,
            wupVector=[0, 0, 1],
            wupObject=self.ankle_ctl,
            maintainOffset=False,
        )

        applyop.aimCns(
            self.lowlegTangentB_loc,
            self.lowlegTangentA_npo,
            axis=axis,
            wupType=2,
            wupVector=[0, 0, 1],
            wupObject=self.ankle_ctl,
            maintainOffset=False,
        )

        pm.pointConstraint(self.tws3_loc, self.lowlegTangentB_loc)

        # Volume -------------------------------------------
        distA_node = node.createDistNode(self.tws0_loc, self.tws1_loc)
        distB_node = node.createDistNode(self.tws1_loc, self.tws2_loc)
        distC_node = node.createDistNode(self.tws2_loc, self.tws3_loc)
        add_node = node.createAddNode(
            distA_node + ".distance", distB_node + ".distance"
        )
        add_node2 = node.createAddNode(
            distC_node + ".distance", add_node + ".output"
        )
        div_node = node.createDivNode(
            add_node2 + ".output", self.root_ctl.attr("sx")
        )

        # comp scaling issue
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(self.root.attr("worldMatrix"), dm_node + ".inputMatrix")

        div_node2 = node.createDivNode(
            div_node + ".outputX", dm_node + ".outputScaleX"
        )

        self.volDriver_att = div_node2 + ".outputX"

        # connecting tangent scale compensation after volume to avoid
        # duplicating nodes
        distA_node = node.createDistNode(self.tws0_loc, self.knee_ctl)
        distB_node = node.createDistNode(self.knee_ctl, self.ankle_ctl)
        distC_node = node.createDistNode(self.ankle_ctl, self.tws3_loc)

        div_nodeUpLeg = node.createDivNode(
            distA_node + ".distance", dm_node.attr("outputScaleX")
        )
        div_node2 = node.createDivNode(
            div_nodeUpLeg + ".outputX", distA_node.attr("distance").get()
        )
        pm.connectAttr(div_node2.attr("outputX"), self.tws1A_loc.attr("sx"))
        pm.connectAttr(
            div_node2.attr("outputX"), self.uplegTangentA_loc.attr("sx")
        )

        div_nodeMidLeg = node.createDivNode(
            distB_node + ".distance", dm_node.attr("outputScaleX")
        )
        div_node2 = node.createDivNode(
            div_nodeMidLeg + ".outputX", distB_node.attr("distance").get()
        )
        pm.connectAttr(div_node2.attr("outputX"), self.tws1B_loc.attr("sx"))
        pm.connectAttr(div_node2.attr("outputX"), self.tws2A_loc.attr("sx"))

        div_nodeLowLeg = node.createDivNode(
            distC_node + ".distance", dm_node.attr("outputScaleX")
        )
        div_node2 = node.createDivNode(
            div_nodeLowLeg + ".outputX", distC_node.attr("distance").get()
        )
        pm.connectAttr(div_node2.attr("outputX"), self.tws2B_loc.attr("sx"))
        pm.connectAttr(
            div_node2.attr("outputX"), self.lowlegTangentB_loc.attr("sx")
        )

        # conection curve
        cnts = [
            self.uplegTangentA_loc,
            self.uplegTangentA_ctl,
            self.uplegTangentB_ctl,
            self.kneeTangent_ctl,
        ]
        applyop.gear_curvecns_op(self.uplegTwistCrv, cnts)

        cnts = [
            self.kneeTangent_ctl,
            self.midlegTangentA_ctl,
            self.midlegTangentB_ctl,
            self.ankleTangent_ctl,
        ]
        applyop.gear_curvecns_op(self.midlegTwistCrv, cnts)

        cnts = [
            self.ankleTangent_ctl,
            self.lowlegTangentA_ctl,
            self.lowlegTangentB_ctl,
            self.lowlegTangentB_loc,
        ]
        applyop.gear_curvecns_op(self.lowlegTwistCrv, cnts)

        # Tangent controls vis
        for shp in self.uplegTangentA_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))
        for shp in self.uplegTangentB_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))
        for shp in self.midlegTangentA_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))
        for shp in self.midlegTangentB_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))
        for shp in self.lowlegTangentA_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))
        for shp in self.lowlegTangentB_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))
        for shp in self.kneeTangent_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))
        for shp in self.ankleTangent_ctl.getShapes():
            pm.connectAttr(self.tangentVis_att, shp.attr("visibility"))

        # Divisions ----------------------------------------
        # at 0 or 1 the division will follow exactly the rotation of the
        # controler.. and we wont have this nice tangent + roll.
        # boundaries: upleg -> [0, div0+2), midleg -> [div0+2, div0+div1+3),
        # lowleg -> [div0+div1+3, divisions)
        uplegEnd = self.settings["div0"] + 2
        midlegEnd = uplegEnd + self.settings["div1"] + 1
        lastUpLegDiv = None
        lastMidLegDiv = None
        lastLowLegDiv = None
        for i, div_cns in enumerate(self.div_cns):
            if i < uplegEnd:
                mulmat_node = applyop.gear_mulmatrix_op(
                    self.uplegTwistChain[i] + ".worldMatrix",
                    div_cns + ".parentInverseMatrix",
                )
                lastUpLegDiv = div_cns
            elif i < midlegEnd:
                o_node = self.midlegTwistChain[i - uplegEnd]
                mulmat_node = applyop.gear_mulmatrix_op(
                    o_node + ".worldMatrix", div_cns + ".parentInverseMatrix"
                )
                lastMidLegDiv = div_cns
            else:
                o_node = self.lowlegTwistChain[i - midlegEnd]
                mulmat_node = applyop.gear_mulmatrix_op(
                    o_node + ".worldMatrix", div_cns + ".parentInverseMatrix"
                )
                lastLowLegDiv = div_cns
            dm_node = node.createDecomposeMatrixNode(mulmat_node + ".output")
            pm.connectAttr(dm_node + ".outputTranslate", div_cns + ".t")
            pm.connectAttr(dm_node + ".outputRotate", div_cns + ".r")

            # Squash n Stretch
            o_node = applyop.gear_squashstretch2_op(
                div_cns, None, pm.getAttr(self.volDriver_att), "x"
            )
            pm.connectAttr(self.volume_att, o_node + ".blend")
            pm.connectAttr(self.volDriver_att, o_node + ".driver")
            pm.connectAttr(self.st_att[i], o_node + ".stretch")
            pm.connectAttr(self.sq_att[i], o_node + ".squash")

        # force translation for last loc of each twist chain
        applyop.gear_mulmatrix_op(
            self.kneeTangent_ctl.worldMatrix,
            lastUpLegDiv.parentInverseMatrix,
            lastUpLegDiv,
            "t",
        )
        applyop.gear_mulmatrix_op(
            self.ankleTangent_ctl.worldMatrix,
            lastMidLegDiv.parentInverseMatrix,
            lastMidLegDiv,
            "t",
        )
        applyop.gear_mulmatrix_op(
            self.tws3_loc.worldMatrix,
            lastLowLegDiv.parentInverseMatrix,
            lastLowLegDiv,
            "t",
        )

        # Visibilities -------------------------------------
        # fk
        fkvis_node = node.createReverseNode(self.blend_att)
        for ctrl in self.fk_ctl:
            for shp in ctrl.getShapes():
                pm.connectAttr(fkvis_node + ".outputX", shp.attr("visibility"))
        # ik
        for ctrl in [self.ik_ctl, self.roll_ctl, self.upv_ctl, self.line_ref]:
            for shp in ctrl.getShapes():
                pm.connectAttr(self.blend_att, shp.attr("visibility"))

        # setup leg o_node scale compensate
        pm.connectAttr(self.rig.global_ctl + ".scale", self.setup + ".scale")

        # match IK/FK ref
        pm.parentConstraint(self.legBones[0], self.match_fk0_off, mo=True)
        pm.parentConstraint(self.legBones[1], self.match_fk1_off, mo=True)
        pm.parentConstraint(self.legBones[2], self.match_fk2_off, mo=True)

        # squash foot
        self.foot_squash_att >> self.squash_scl.sx
        self.foot_squash_att >> self.squash_scl.sz
        rev_nod = node.createReverseNode(self.foot_squash_att)
        add_node = node.createPlusMinusAverage1D(
            [rev_nod.outputX, 1], output=self.squash_scl.sy
        )

    # =====================================================
    # CONNECTOR
    # =====================================================

    def setRelation(self):
        """Set the relation beetween object from guide to rig"""
        self.relatives["root"] = self.div_cns[0]
        self.relatives["knee"] = self.div_cns[self.settings["div0"] + 2]
        self.relatives["ankle"] = self.div_cns[
            self.settings["div0"] + self.settings["div1"] + 3
        ]
        self.relatives["foot"] = self.div_cns[-1]
        self.relatives["eff"] = self.squash_ref

        self.controlRelatives["root"] = self.fk0_ctl
        self.controlRelatives["knee"] = self.fk1_ctl
        self.controlRelatives["ankle"] = self.fk2_ctl
        self.controlRelatives["foot"] = self.ik_ctl
        self.controlRelatives["eff"] = self.fk3_ctl

        self.jointRelatives["root"] = 0
        self.jointRelatives["knee"] = self.settings["div0"] + 2
        self.jointRelatives["ankle"] = (
            self.settings["div0"] + self.settings["div1"] + 3
        )
        self.jointRelatives["foot"] = len(self.div_cns)
        self.jointRelatives["eff"] = len(self.div_cns)

        self.aliasRelatives["eff"] = "tip"

    # standard connection definition.
    def connect_standard(self):
        self.parent.addChild(self.root)

        # Set the Ik Reference
        self.connectRef(self.settings["ikrefarray"], self.ik_cns)
        if self.settings["upvrefarray"]:
            self.connectRef(
                "Auto," + self.settings["upvrefarray"], self.upv_cns, True
            )

        # Pin the knee / ankle. "Auto" keeps them following the mid joints
        # (the default parentConstraint on knee_lvl / ankle_lvl).
        if self.settings["kneerefarray"]:
            self.connectRef2(
                "Auto," + self.settings["kneerefarray"],
                self.knee_cns,
                self.kneeref_att,
                [self.knee_lvl],
                False,
            )
        if self.settings["anklerefarray"]:
            self.connectRef2(
                "Auto," + self.settings["anklerefarray"],
                self.ankle_cns,
                self.ankleref_att,
                [self.ankle_lvl],
                False,
            )
