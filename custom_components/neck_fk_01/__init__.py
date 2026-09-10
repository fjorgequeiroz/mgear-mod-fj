"""Component Neck FK 01 module.

This component is the FK-primary counterpart of ``neck_ik_01``.

In ``neck_ik_01`` the IK control at the top of the neck is the master: it
drives a master curve, a curve-slide solver, a slave curve and a chain of
path constrained ``div_cns`` transforms, and the "FK" controls are just the
visible read-out of that solve (they are downstream, non-master transforms).

In ``neck_fk_01`` the dependency is reversed. The ``fk*_ctl`` chain is a real,
parented FK hierarchy and is the only input. The ``ik_ctl`` is kept for
familiarity and for anything that expects a neck "ik" handle, but it is a
locked, non-keyable *output*: it is matrix driven by the last FK control so it
always sits at the tip of the neck and follows the chain.

Cyclic redundancy is avoided by construction:

* There is no IK spline: no ``gear_curveslide2_op``, no ``pathCns`` and no
  ``gear_intmatrix_op`` reading ``ik_ctl.worldMatrix``. Those operators are
  what made the IK control the master in ``neck_ik_01``.
* ``ik_ctl`` never feeds back into anything the FK chain reads. The only
  connection is ``fk_ctl[-1].worldMatrix -> ik_cns`` (one direction).
* The squash and stretch writes only to the leaf ``scl_ref`` transforms.
  Those have no children, so scaling them moves no control. The live
  chain-length measurement can therefore read the ``fk_ctl`` world positions
  without the squash output feeding back into that measurement (which is the
  classic stretchy-FK cycle, and how the first draft of this component broke).
* The optional display curve also reads FK-driven transforms only.
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

        # base orientation for the whole neck --------------
        t_base = transform.getTransformLookingAt(
            self.guide.pos["root"],
            self.guide.pos["neck"],
            self.normal,
            "yx",
            self.negate,
        )

        # FK Controlers (the master chain) ----------------
        # First and last positions are an obligation, the user only defines
        # how many intermediate divisions.
        self.fk_ctl = []
        self.fk_npo = []
        self.scl_ref = []

        parentctl = self.root
        self.previousCtlTag = self.parentCtlTag

        # sample positions along the guide root -> neck segment
        self.fk_pos = [
            vector.linearlyInterpolate(
                self.guide.pos["root"],
                self.guide.pos["neck"],
                i / (self.divisions - 1.0),
            )
            for i in range(self.divisions)
        ]

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

            # Leaf scale reference. The squash and stretch writes here and
            # NOWHERE else. Because it has no children, scaling it moves no
            # control, so the live chain-length measurement below can safely
            # read the fk_ctl world positions without creating a cycle.
            scl_ref = primitive.addTransform(
                fk_ctl,
                self.getName("%s_scl_ref" % i),
                transform.getTransform(fk_ctl),
            )
            self.scl_ref.append(scl_ref)

            # Deform joint rides the scale reference (position/orient from the
            # fk_ctl, scale from the squash and stretch).
            self.jnt_pos.append([scl_ref, i])

            parentctl = fk_ctl

        # IK read-out control (driven output, never an input) ---------
        if self.settings["IKWorldOri"]:
            t_ik = transform.setMatrixPosition(
                datatypes.TransformationMatrix(), self.guide.pos["neck"]
            )
        else:
            t_ik = transform.setMatrixPosition(t_base, self.guide.pos["neck"])

        self.ik_cns = primitive.addTransform(
            self.root, self.getName("ik_cns"), t_ik
        )

        self.ik_ctl = self.addCtl(
            self.ik_cns,
            "ik_ctl",
            t_ik,
            self.color_ik,
            "compas",
            w=self.size * 0.5,
            tp=self.previousCtlTag,
        )
        attribute.setRotOrder(self.ik_ctl, "ZXY")
        # It is a visual read-out of the FK tip: lock every transform channel
        # (but not visibility) so nothing (animator, ref array, parent) can
        # push it back into the rig and create a cycle.
        attribute.lockAttribute(
            self.ik_ctl,
            ["tx", "ty", "tz", "rx", "ry", "rz", "sx", "sy", "sz"],
        )

        # Tangent display locators (optional, cosmetic) --------------
        if self.settings["tangentControls"]:
            t = transform.setMatrixPosition(t_base, self.guide.pos["tan1"])
            self.tan1_loc = primitive.addTransform(
                self.fk_ctl[-1], self.getName("tan1_loc"), t
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

            crv_ends = [self.tan0_ctl, self.tan1_ctl]
        else:
            t = transform.setMatrixPosition(t_base, self.guide.pos["tan1"])
            self.tan1_loc = primitive.addTransform(
                self.fk_ctl[-1], self.getName("tan1_loc"), t
            )
            t = transform.setMatrixPosition(t_base, self.guide.pos["tan0"])
            self.tan0_loc = primitive.addTransform(
                self.fk_ctl[0], self.getName("tan0_loc"), t
            )
            crv_ends = [self.tan0_loc, self.tan1_loc]

        # Display curve through the FK chain (visual only) -----------
        self.dsp_crv = curve.addCnsCurve(
            self.root,
            self.getName("dsp_crv"),
            [self.fk_ctl[0]] + crv_ends + [self.ik_ctl],
            3,
        )
        self.dsp_crv.setAttr("visibility", False)

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
        # Volume (drives the optional squash and stretch)
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 0, 0, 1
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
        # Squash and stretch profile
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

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        """Create operators and set the relations for the component rig.

        Apply operators, constraints, expressions to the hierarchy.
        In order to keep the code clean and easier to debug,
        we shouldn't create any new object in this method.
        """

        # IK read-out ------------------------------------
        # ik_cns rigidly follows the last FK control. This is the ONLY link
        # between the FK chain and the ik_ctl and it goes one way only, so no
        # cycle is possible.
        mulmat_node = applyop.gear_mulmatrix_op(
            self.fk_ctl[-1].attr("worldMatrix"),
            self.ik_cns.attr("parentInverseMatrix[0]"),
        )
        dm_node = node.createDecomposeMatrixNode(mulmat_node + ".output")
        pm.connectAttr(dm_node + ".outputTranslate", self.ik_cns.attr("t"))
        pm.connectAttr(dm_node + ".outputRotate", self.ik_cns.attr("r"))
        pm.connectAttr(dm_node + ".outputScale", self.ik_cns.attr("s"))

        # Squash and stretch ----------------------------
        # Live length of the FK chain.
        #
        # No cycle: the squash and stretch only writes to the leaf scl_ref
        # transforms (which have no children), so it never moves an fk_ctl.
        # The distanceBetween nodes therefore read fk_ctl world positions that
        # the squash output cannot feed back into.
        rootWorld_node = node.createDecomposeMatrixNode(
            self.root.attr("worldMatrix")
        )
        rest_length = 0.0
        length_plugs = []
        for i in range(self.divisions - 1):
            rest_length += vector.getDistance(
                self.fk_pos[i], self.fk_pos[i + 1]
            )
            seg_dist = node.createDistNode(
                self.fk_ctl[i], self.fk_ctl[i + 1]
            )
            # normalise out the global rig scale
            seg_div = node.createDivNode(
                seg_dist + ".distance", rootWorld_node + ".outputScaleX"
            )
            length_plugs.append(seg_div + ".outputX")

        length_node = node.createPlusMinusAverage1D(length_plugs)

        for i in range(self.divisions):
            op = applyop.gear_squashstretch2_op(
                self.scl_ref[i],
                self.root,
                rest_length,
                "y",
            )
            pm.connectAttr(self.volume_att, op + ".blend")
            pm.connectAttr(length_node + ".output1D", op + ".driver")
            pm.connectAttr(self.st_att[i], op + ".stretch")
            pm.connectAttr(self.sq_att[i], op + ".squash")
            op.setAttr("driver_min", rest_length * 0.1)

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        """Set the relation beetween object from guide to rig"""
        self.relatives["root"] = self.fk_ctl[0]
        self.relatives["tan1"] = self.fk_ctl[-1]
        self.relatives["tan2"] = self.head_ctl
        self.relatives["neck"] = self.head_ctl
        self.relatives["head"] = self.head_ctl
        self.relatives["eff"] = self.head_ctl

        self.controlRelatives["root"] = self.fk_ctl[0]
        self.controlRelatives["tan1"] = self.fk_ctl[-1]
        self.controlRelatives["tan2"] = self.head_ctl
        self.controlRelatives["neck"] = self.head_ctl
        self.controlRelatives["head"] = self.head_ctl
        self.controlRelatives["eff"] = self.head_ctl

        self.jointRelatives["root"] = 0
        self.jointRelatives["tan1"] = self.divisions - 1
        self.jointRelatives["tan2"] = len(self.jnt_pos) - 1
        self.jointRelatives["neck"] = len(self.jnt_pos) - 1
        self.jointRelatives["head"] = len(self.jnt_pos) - 1
        self.jointRelatives["eff"] = len(self.jnt_pos) - 1

        self.aliasRelatives["tan1"] = "neck"
        self.aliasRelatives["tan2"] = "head"
        self.aliasRelatives["neck"] = "head"
        self.aliasRelatives["eff"] = "head"

    def connect_standard(self):
        self.connect_standardWithFkRef()

    def connect_standardWithFkRef(self):
        self.parent.addChild(self.root)

        # Neck base space switch. It drives the first fk npo, which sits
        # above every FK control, so the whole neck inherits the space
        # without touching the ik_ctl read-out.
        if self.settings["fkrefarray"] and hasattr(self, "fkref_att"):
            ref_names = self.get_valid_ref_list(
                self.settings["fkrefarray"].split(",")
            )
            if len(ref_names) >= 1:
                ref = []
                for ref_name in ref_names:
                    ref.append(self.rig.findRelative(ref_name))
                ref.append(self.fk_npo[0])
                cns_node = pm.parentConstraint(
                    *ref, skipTranslate=["x", "y", "z"], maintainOffset=True
                )
                cns_attr_names = pm.parentConstraint(
                    cns_node, query=True, weightAliasList=True
                )
                cns_attr = [
                    "{}.{}".format(cns_node, cname)
                    for cname in cns_attr_names
                ]
                for i, attr in enumerate(cns_attr):
                    node_name = pm.createNode("condition")
                    pm.connectAttr(self.fkref_att, node_name + ".firstTerm")
                    pm.setAttr(node_name + ".secondTerm", i + 1)
                    pm.setAttr(node_name + ".operation", 0)
                    pm.setAttr(node_name + ".colorIfTrueR", 1)
                    pm.setAttr(node_name + ".colorIfFalseR", 0)
                    pm.connectAttr(node_name + ".outColorR", attr)

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
