"""Component Neck FK 01 module.

FK-primary variant of ``neck_ik_01``.

``neck_ik_01`` keeps the full IK-spline solve (master curve -> curve-slide ->
slave curve -> path-constrained ``div_cns`` -> deform joints) and the IK
control at the top of the neck is the master of that solve. Its "fk" controls
are fake: they are the visible read-out of the spline.

``neck_fk_01`` keeps *exactly the same IK-spline solve and the same joint
output*, but reverses who is in charge:

* The ``fk*_ctl`` chain is a real, parented FK hierarchy and is the master.
* The ``ik_ctl`` and the tangent controls are **parented under the FK chain**
  (``ik_cns`` under ``fk_ctl[-1]``, ``tan0`` under ``fk_ctl[0]``). Rotating or
  translating an FK control carries its child IK / tangent controls with it
  through plain DAG parenting - no connection, no constraint.
* The IK / tangent controls stay fully keyable, so an animator can still add
  an IK-style offset *on top of* the FK pose (same idea as
  ``spine_FK_01_horizontal``).

Why there is no cycle
---------------------
The dependency chain is strictly one way::

    fk_ctl  --(DAG parent)-->  ik_ctl / tan_ctl
            --(mst_crv CV cns)-->  gear_curveslide2 -> slv_crv
            --(pathCns)-->  div_cns   (inheritsTransform = False, so it is
                                       NOT under the FK hierarchy; it only
                                       moves via the path constraint)
            -->  scl_ref (leaf)  -->  deform joint

Nothing downstream of ``div_cns`` is ever read back by the FK chain or by the
curves, and the squash & stretch only writes to the leaf ``scl_ref``
transforms, so it moves no control. This is the same structure
``neck_ik_01`` uses; only the ownership of the IK / tangent controls changed.
"""

import mgear.pymaya as pm
from mgear.pymaya import datatypes

from mgear.shifter import component

from mgear.core import node, fcurve, applyop, vector, curve
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
        self.divisions = self.settings["division"]

        # base orientation of the neck --------------------
        t_base = transform.getTransformLookingAt(
            self.guide.pos["root"],
            self.guide.pos["neck"],
            self.normal,
            "yx",
            self.negate,
        )

        # sampled positions root -> neck for the FK controls
        self.fk_pos = [
            vector.linearlyInterpolate(
                self.guide.pos["root"],
                self.guide.pos["neck"],
                i / (self.divisions - 1.0),
            )
            for i in range(self.divisions)
        ]

        # FK Controlers (the master chain) ----------------
        self.fk_ctl = []
        self.fk_npo = []
        parentctl = self.root
        self.previousCtlTag = self.parentCtlTag

        for i in range(self.divisions):
            t = transform.setMatrixPosition(t_base, self.fk_pos[i])

            fk_npo = primitive.addTransform(
                parentctl, self.getName("fk%s_npo" % i), t
            )
            self.fk_npo.append(fk_npo)

            fk_ctl = self.addCtl(
                fk_npo,
                "fk%s_ctl" % i,
                t,
                self.color_fk,
                "cube",
                w=self.size * 0.2,
                h=self.size * 0.05,
                d=self.size * 0.2,
                tp=self.previousCtlTag,
            )
            self.fk_ctl.append(fk_ctl)
            self.previousCtlTag = fk_ctl

            attribute.setKeyableAttributes(fk_ctl)
            attribute.setRotOrder(fk_ctl, "ZXY")
            attribute.setInvertMirror(fk_ctl, ["tx", "rz", "ry"])

            parentctl = fk_ctl

        # Ik Controler (child of the FK tip: FK drives it) ------------
        if self.settings["IKWorldOri"]:
            t = transform.setMatrixPosition(
                datatypes.TransformationMatrix(), self.guide.pos["neck"]
            )
        else:
            t = transform.getTransformLookingAt(
                self.guide.pos["tan1"],
                self.guide.pos["neck"],
                self.normal,
                "yx",
                self.negate,
            )
            t = transform.setMatrixPosition(t, self.guide.pos["neck"])

        self.ik_cns = primitive.addTransform(
            self.fk_ctl[-1], self.getName("ik_cns"), t
        )

        self.ik_ctl = self.addCtl(
            self.ik_cns,
            "ik_ctl",
            t,
            self.color_ik,
            "compas",
            w=self.size * 0.5,
            tp=self.previousCtlTag,
        )

        attribute.setKeyableAttributes(self.ik_ctl, self.tr_params)
        attribute.setRotOrder(self.ik_ctl, "ZXY")
        attribute.setInvertMirror(self.ik_ctl, ["tx", "ry", "rz"])

        # Tangents (also children of the FK chain) -------------------
        if self.settings["tangentControls"]:
            t = transform.setMatrixPosition(t, self.guide.pos["tan1"])

            self.tan1_loc = primitive.addTransform(
                self.ik_ctl, self.getName("tan1_loc"), t
            )
            self.tan1_ctl = self.addCtl(
                self.tan1_loc,
                "tan1_ctl",
                t,
                self.color_ik,
                "sphere",
                w=self.size * 0.2,
                tp=self.ik_ctl,
            )
            attribute.setKeyableAttributes(self.tan1_ctl, self.t_params)
            attribute.setInvertMirror(self.tan1_ctl, ["tx"])

            t = transform.getTransformLookingAt(
                self.guide.pos["root"],
                self.guide.pos["tan0"],
                self.normal,
                "yx",
                self.negate,
            )
            t = transform.setMatrixPosition(t, self.guide.pos["tan0"])

            self.tan0_loc = primitive.addTransform(
                self.fk_ctl[0], self.getName("tan0_loc"), t
            )
            self.tan0_ctl = self.addCtl(
                self.tan0_loc,
                "tan0_ctl",
                t,
                self.color_ik,
                "sphere",
                w=self.size * 0.2,
                tp=self.ik_ctl,
            )
            attribute.setKeyableAttributes(self.tan0_ctl, self.t_params)
            attribute.setInvertMirror(self.tan0_ctl, ["tx"])

            self.mst_crv = curve.addCnsCurve(
                self.root,
                self.getName("mst_crv"),
                [self.fk_ctl[0], self.tan0_ctl, self.tan1_ctl, self.ik_ctl],
                3,
            )
        else:
            t = transform.setMatrixPosition(t, self.guide.pos["tan1"])
            self.tan1_loc = primitive.addTransform(
                self.ik_ctl, self.getName("tan1_loc"), t
            )

            t = transform.getTransformLookingAt(
                self.guide.pos["root"],
                self.guide.pos["tan0"],
                self.normal,
                "yx",
                self.negate,
            )
            t = transform.setMatrixPosition(t, self.guide.pos["tan0"])

            self.tan0_loc = primitive.addTransform(
                self.fk_ctl[0], self.getName("tan0_loc"), t
            )

            self.mst_crv = curve.addCnsCurve(
                self.root,
                self.getName("mst_crv"),
                [self.fk_ctl[0], self.tan0_loc, self.tan1_loc, self.ik_ctl],
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

        # Division -----------------------------------------
        # First and last divisions are an obligation, the user only defines
        # the intermediate ones. div_cns are NOT under the FK hierarchy; they
        # only move along the slave curve via the path constraint.
        parentdiv = self.root
        self.div_cns = []
        self.scl_ref = []
        self.twister = []
        self.ref_twist = []

        parent_twistRef = primitive.addTransform(
            self.root,
            self.getName("reference"),
            transform.getTransform(self.root),
        )

        t = transform.getTransformLookingAt(
            self.guide.pos["root"],
            self.guide.pos["neck"],
            self.normal,
            "yx",
            self.negate,
        )

        self.intMRef = primitive.addTransform(
            self.root, self.getName("intMRef"), t
        )

        for i in range(self.divisions):
            div_cns = primitive.addTransform(
                parentdiv, self.getName("%s_cns" % i), t
            )
            pm.setAttr(div_cns + ".inheritsTransform", False)
            self.div_cns.append(div_cns)
            parentdiv = div_cns

            # Leaf scale reference under the div_cns. The squash & stretch
            # writes only here, and it has no children, so it moves no
            # control and cannot feed back into the length measurement.
            scl_ref = primitive.addTransform(
                div_cns,
                self.getName("%s_scl_ref" % i),
                transform.getTransform(div_cns),
            )
            self.scl_ref.append(scl_ref)

            # deform joint rides the scale reference
            self.jnt_pos.append([scl_ref, i])

            # twist references (replace the spinlookup slerp solver)
            twister = primitive.addTransform(
                parent_twistRef, self.getName("%s_rot_ref" % i), t
            )
            ref_twist = primitive.addTransform(
                parent_twistRef, self.getName("%s_pos_ref" % i), t
            )
            ref_twist.setTranslation(
                datatypes.Vector(0.0, 0, 1.0), space="preTransform"
            )
            self.twister.append(twister)
            self.ref_twist.append(ref_twist)

        # Head --------------------------------------------
        t = transform.getTransformLookingAt(
            self.guide.pos["head"],
            self.guide.pos["eff"],
            self.normal,
            "yx",
            self.negate,
        )

        self.head_cns = primitive.addTransform(
            self.fk_ctl[-1], self.getName("head_cns"), t
        )

        dist = vector.getDistance(
            self.guide.pos["head"], self.guide.pos["eff"]
        )

        self.head_ctl = self.addCtl(
            self.head_cns,
            "head_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.size * 0.5,
            h=dist,
            d=self.size * 0.5,
            po=datatypes.Vector(0, dist * 0.5, 0),
            tp=self.previousCtlTag,
        )

        attribute.setRotOrder(self.head_ctl, "ZXY")
        attribute.setInvertMirror(self.head_ctl, ["tx", "rz", "ry"])

        self.jnt_pos.append([self.head_ctl, "head"])

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        """Create the anim and setup rig attributes for the component"""

        # Anim -------------------------------------------
        self.maxstretch_att = self.addAnimParam(
            "maxstretch",
            "Max Stretch",
            "double",
            self.settings["maxstretch"],
            1,
        )
        self.maxsquash_att = self.addAnimParam(
            "maxsquash", "MaxSquash", "double", self.settings["maxsquash"], 0, 1
        )
        self.softness_att = self.addAnimParam(
            "softness", "Softness", "double", self.settings["softness"], 0, 1
        )
        self.lock_ori_att = self.addAnimParam(
            "lock_ori", "Lock Ori", "double", 1, 0, 1
        )
        self.tan0_att = self.addAnimParam("tan0", "Tangent 0", "double", 1, 0)
        self.tan1_att = self.addAnimParam("tan1", "Tangent 1", "double", 1, 0)

        # Volume
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )

        # Head space switch
        if self.settings["headrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["headrefarray"].split(",")
            )
            if len(ref_names) > 1:
                ref_names.insert(0, "self")
                self.headref_att = self.addAnimEnumParam(
                    "headref", "Head Ref", 0, ref_names
                )

        # Neck base space switch
        if self.settings["fkrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["fkrefarray"].split(",")
            )
            if len(ref_names) > 1:
                ref_names.insert(0, "self")
                self.fkref_att = self.addAnimEnumParam(
                    "fkref", "FK Ref", 0, ref_names
                )

        # Setup ------------------------------------------
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
                "stretch_%s" % i, "Stretch %s" % i, "double",
                self.st_value[i], -1, 0,
            )
            for i in range(self.divisions)
        ]
        self.sq_att = [
            self.addSetupParam(
                "squash_%s" % i, "Squash %s" % i, "double",
                self.sq_value[i], 0, 1,
            )
            for i in range(self.divisions)
        ]

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        """Create operators and set the relations for the component rig.

        Apply operators, constraints, expressions to the hierarchy.
        In order to keep the code clean and easier to debug,
        we shouldn't create any new object in this method.
        """

        # Tangent position --------------------------------
        d = vector.getDistance(self.guide.pos["root"], self.guide.pos["neck"])
        dist_node = node.createDistNode(self.fk_ctl[0], self.ik_ctl)
        rootWorld_node = node.createDecomposeMatrixNode(
            self.root.attr("worldMatrix")
        )
        div_node = node.createDivNode(
            dist_node + ".distance", rootWorld_node + ".outputScaleX"
        )
        div_node = node.createDivNode(div_node + ".outputX", d)

        # tan0
        mul_node = node.createMulNode(
            self.tan0_att, self.tan0_loc.getAttr("ty")
        )
        res_node = node.createMulNode(
            mul_node + ".outputX", div_node + ".outputX"
        )
        pm.connectAttr(res_node + ".outputX", self.tan0_loc + ".ty")

        # tan1
        mul_node = node.createMulNode(
            self.tan1_att, self.tan1_loc.getAttr("ty")
        )
        res_node = node.createMulNode(
            mul_node + ".outputX", div_node + ".outputX"
        )
        pm.connectAttr(res_node + ".outputX", self.tan1_loc.attr("ty"))

        # Curves ------------------------------------------
        op = applyop.gear_curveslide2_op(
            self.slv_crv, self.mst_crv, 0, 1.5, 0.5, 0.5
        )
        pm.connectAttr(self.maxstretch_att, op + ".maxstretch")
        pm.connectAttr(self.maxsquash_att, op + ".maxsquash")
        pm.connectAttr(self.softness_att, op + ".softness")

        # Volume driver -----------------------------------
        crv_node = node.createCurveInfoNode(self.slv_crv)

        # Division ----------------------------------------
        for i in range(self.divisions):
            u = i / (self.divisions - 1.0)

            cns = applyop.pathCns(
                self.div_cns[i], self.slv_crv, False, u, True
            )
            cns.setAttr("frontAxis", 1)  # front axis is 'Y'
            cns.setAttr("upAxis", 2)     # up axis is 'Z'

            # Roll
            intMatrix = applyop.gear_intmatrix_op(
                self.intMRef + ".worldMatrix", self.ik_ctl + ".worldMatrix", u
            )
            dm_node = node.createDecomposeMatrixNode(intMatrix + ".output")
            pm.connectAttr(
                dm_node + ".outputRotate", self.twister[i].attr("rotate")
            )
            pm.parentConstraint(
                self.twister[i], self.ref_twist[i], maintainOffset=True
            )
            pm.connectAttr(
                self.ref_twist[i] + ".translate", cns + ".worldUpVector"
            )

            # Squash & stretch (writes to the leaf scl_ref only)
            op = applyop.gear_squashstretch2_op(
                self.scl_ref[i],
                self.root,
                pm.arclen(self.slv_crv),
                "y",
            )
            pm.connectAttr(self.volume_att, op + ".blend")
            pm.connectAttr(crv_node + ".arcLength", op + ".driver")
            pm.connectAttr(self.st_att[i], op + ".stretch")
            pm.connectAttr(self.sq_att[i], op + ".squash")
            op.setAttr("driver_min", 0.1)

            # Orientation Lock on the last division
            if i == self.divisions - 1:
                dm_node = node.createDecomposeMatrixNode(
                    self.ik_ctl + ".worldMatrix"
                )
                blend_node = node.createBlendNode(
                    [dm_node + ".outputRotate%s" % s for s in "XYZ"],
                    [cns + ".rotate%s" % s for s in "XYZ"],
                    self.lock_ori_att,
                )
                self.div_cns[i].attr("rotate").disconnect()
                pm.connectAttr(
                    blend_node + ".output", self.div_cns[i] + ".rotate"
                )

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        """Set the relation beetween object from guide to rig"""
        self.relatives["root"] = self.scl_ref[0]
        self.relatives["tan1"] = self.scl_ref[0]
        self.relatives["tan2"] = self.head_ctl
        self.relatives["neck"] = self.head_ctl
        self.relatives["head"] = self.head_ctl
        self.relatives["eff"] = self.head_ctl

        self.controlRelatives["root"] = self.fk_ctl[0]
        self.controlRelatives["tan1"] = self.head_ctl
        self.controlRelatives["tan2"] = self.head_ctl
        self.controlRelatives["neck"] = self.head_ctl
        self.controlRelatives["head"] = self.head_ctl
        self.controlRelatives["eff"] = self.head_ctl

        self.jointRelatives["root"] = 0
        self.jointRelatives["tan1"] = 0
        self.jointRelatives["tan2"] = len(self.jnt_pos) - 1
        self.jointRelatives["neck"] = len(self.jnt_pos) - 1
        self.jointRelatives["head"] = len(self.jnt_pos) - 1
        self.jointRelatives["eff"] = len(self.jnt_pos) - 1

        self.aliasRelatives["tan1"] = "root"
        self.aliasRelatives["tan2"] = "head"
        self.aliasRelatives["neck"] = "head"
        self.aliasRelatives["eff"] = "head"

    def connect_standard(self):
        self.connect_standardWithFkRef()

    def connect_standardWithFkRef(self):
        self.parent.addChild(self.root)

        # Neck base space switch: drives fk0_npo (above every control), so the
        # whole neck - including its child IK / tangent controls - inherits
        # the space. Never touches div_cns / the joints directly.
        if self.settings["fkrefarray"] and hasattr(self, "fkref_att"):
            ref_names = self.get_valid_ref_list(
                self.settings["fkrefarray"].split(",")
            )
            if len(ref_names) >= 1:
                ref = [self.rig.findRelative(n) for n in ref_names]
                ref.append(self.fk_npo[0])
                cns_node = pm.parentConstraint(
                    *ref, skipTranslate=["x", "y", "z"], maintainOffset=True
                )
                cns_attr_names = pm.parentConstraint(
                    cns_node, query=True, weightAliasList=True
                )
                cns_attr = [
                    "{}.{}".format(cns_node, n) for n in cns_attr_names
                ]
                for i, attr in enumerate(cns_attr):
                    cond = pm.createNode("condition")
                    pm.connectAttr(self.fkref_att, cond + ".firstTerm")
                    pm.setAttr(cond + ".secondTerm", i + 1)
                    pm.setAttr(cond + ".operation", 0)
                    pm.setAttr(cond + ".colorIfTrueR", 1)
                    pm.setAttr(cond + ".colorIfFalseR", 0)
                    pm.connectAttr(cond + ".outColorR", attr)

        # Head space switch (same behaviour as neck_ik_01)
        if self.settings["headrefarray"]:
            ref_names = self.settings["headrefarray"].split(",")

            ref = []
            for ref_name in ref_names:
                ref.append(self.rig.findRelative(ref_name))

            ref.append(self.head_cns)
            cns_node = pm.parentConstraint(
                *ref, skipTranslate="none", maintainOffset=True
            )

            cns_attr_names = pm.parentConstraint(
                cns_node, query=True, weightAliasList=True
            )
            cns_attr = []
            for cname in cns_attr_names:
                cns_attr.append("{}.{}".format(cns_node, cname))
            self.head_cns.attr("tx").disconnect()
            self.head_cns.attr("ty").disconnect()
            self.head_cns.attr("tz").disconnect()

            for i, attr in enumerate(cns_attr):
                node_name = pm.createNode("condition")
                pm.connectAttr(self.headref_att, node_name + ".firstTerm")
                pm.setAttr(node_name + ".secondTerm", i + 1)
                pm.setAttr(node_name + ".operation", 0)
                pm.setAttr(node_name + ".colorIfTrueR", 1)
                pm.setAttr(node_name + ".colorIfFalseR", 0)
                pm.connectAttr(node_name + ".outColorR", attr)
