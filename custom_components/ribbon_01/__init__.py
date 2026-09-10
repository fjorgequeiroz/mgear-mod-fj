"""Component Ribbon 01 module.

A ribbon / surface rig.

A NURBS surface is skinned to a chain of FK controllers. Any number of deform
joints then ride the surface (via ``pointOnSurfaceInfo`` -> ``fourByFourMatrix``
-> ``decomposeMatrix``, the classic "rivet" attach), so a low count of FK
controls drives a smooth, dense joint chain - ideal for tails, tentacles,
ropes, straps, antennae, cartoon limbs, lips, etc.

Two independent counts are exposed in the settings:

* **FK Controllers** - how many animation controls drive the ribbon.
* **Deform Joints** - how many joints are attached to the surface.

The guide is a simple multi-locator chain (``root`` + ``#_loc``) plus a blade
for the surface normal / up direction.
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
        self.WIP = self.options["mode"]

        # positions along the guide, evenly by arc length -----------
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
        self.fk_t = transform.getChainTransform(
            self.fk_pos
            + [self.fk_pos[-1] + (self.fk_pos[-1] - self.fk_pos[-2])],
            self.normal,
            self.negate,
            axis="xy",
        )

        # base space-switch null (target of the ikrefarray connector)
        self.ik_cns = primitive.addTransform(
            self.root, self.getName("ik_cns"), self.fk_t[0]
        )

        # FK controllers ---------------------------------
        self.fk_npo = []
        self.fk_ctl = []
        self.fk_ref = []
        self.previousTag = self.parentCtlTag
        parent = self.ik_cns
        for i in range(self.fk_number):
            t = self.fk_t[i]
            if i < self.fk_number - 1:
                dist = vector.getDistance(self.fk_pos[i], self.fk_pos[i + 1])
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

            # the surface is skinned to this reference, one per FK control
            fk_ref = primitive.addTransform(
                fk_ctl, self.getName("fk%s_ref" % i), t
            )

            self.fk_npo.append(fk_npo)
            self.fk_ctl.append(fk_ctl)
            self.fk_ref.append(fk_ref)
            self.previousTag = fk_ctl
            parent = fk_ctl

        # Ribbon surface --------------------------------
        # A degree 3 (length) x degree 1 (width) NURBS plane, one span per
        # FK control, laid on the guide and skinned to the fk_ref chain.
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
            width=self.length if self.length else 1.0,
            lengthRatio=(width / self.length) if self.length else 0.2,
            degree=3,
            patchesU=max(1, self.fk_number - 1),
            patchesV=1,
            axis=[0, 1, 0],
            constructionHistory=False,
        )[0]

        # degree 3 along the length (U), linear across the width (V),
        # parameter range reset to 0-1 in both directions
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

        # place it on the oriented ribbon_root (local +X = chain direction)
        pm.parent(self.surface, self.ribbon_root, relative=True)
        # the skinCluster fully drives the surface from the fk_ref world
        # positions, so the transform must not inherit as well (double move)
        pm.setAttr(self.surface + ".inheritsTransform", False)
        attribute.lockAttribute(
            self.surface, ["tx", "ty", "tz", "rx", "ry", "rz", "sx", "sy", "sz"]
        )

        # skin the surface to the fk references
        self.surface_skin = pm.skinCluster(
            self.fk_ref,
            self.surface,
            name=self.getName("ribbon_skinCluster"),
            toSelectedBones=True,
            bindMethod=0,
            skinMethod=0,
            normalizeWeights=1,
            maximumInfluences=2,
            dropoffRate=4,
        )

        # Deform joints attached to the surface ----------
        self.attach_npo = []
        self.attach = []
        self.attach_u = [
            i / (self.jnt_number - 1.0) for i in range(self.jnt_number)
        ]
        for i in range(self.jnt_number):
            att_npo = primitive.addTransform(
                self.ribbon_root, self.getName("attach%s_npo" % i)
            )
            pm.setAttr(att_npo + ".inheritsTransform", False)

            att = primitive.addTransform(
                att_npo, self.getName("attach%s" % i)
            )

            self.attach_npo.append(att_npo)
            self.attach.append(att)

            self.jnt_pos.append(
                {
                    "obj": att,
                    "name": i,
                    "UniScale": True,
                }
            )

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        """Create the anim and setup rig attributes for the component"""

        self.surfaceVis_att = self.addAnimParam(
            "surface_vis", "Surface Vis", "bool", False
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
        """Create operators and set the relations for the component rig.

        Apply operators, constraints, expressions to the hierarchy.
        """

        srf_shape = self.surface.getShape()
        dm_scl = node.createDecomposeMatrixNode(
            self.root.attr("worldMatrix[0]")
        )

        # the surface was rebuilt with a 0-1 parameter range in both
        # directions, so the attach U value is just the normalized position
        for i, att in enumerate(self.attach):
            u = self.attach_u[i]

            posi = pm.createNode(
                "pointOnSurfaceInfo", name=self.getName("attach%s_posi" % i)
            )
            posi.attr("turnOnPercentage").set(False)
            posi.attr("parameterU").set(u)
            posi.attr("parameterV").set(0.5)
            pm.connectAttr(
                srf_shape.attr("worldSpace[0]"), posi.attr("inputSurface")
            )

            mtx = pm.createNode(
                "fourByFourMatrix", name=self.getName("attach%s_m4x4" % i)
            )
            # U tangent -> X (along the ribbon length, matches the FK chain)
            pm.connectAttr(posi.attr("normalizedTangentUX"), mtx.attr("in00"))
            pm.connectAttr(posi.attr("normalizedTangentUY"), mtx.attr("in01"))
            pm.connectAttr(posi.attr("normalizedTangentUZ"), mtx.attr("in02"))
            # normal -> Y (off the surface, up)
            pm.connectAttr(posi.attr("normalizedNormalX"), mtx.attr("in10"))
            pm.connectAttr(posi.attr("normalizedNormalY"), mtx.attr("in11"))
            pm.connectAttr(posi.attr("normalizedNormalZ"), mtx.attr("in12"))
            # V tangent -> Z (across the ribbon width)
            pm.connectAttr(posi.attr("normalizedTangentVX"), mtx.attr("in20"))
            pm.connectAttr(posi.attr("normalizedTangentVY"), mtx.attr("in21"))
            pm.connectAttr(posi.attr("normalizedTangentVZ"), mtx.attr("in22"))
            # position -> translation row
            pm.connectAttr(posi.attr("positionX"), mtx.attr("in30"))
            pm.connectAttr(posi.attr("positionY"), mtx.attr("in31"))
            pm.connectAttr(posi.attr("positionZ"), mtx.attr("in32"))

            # attach_npo has inheritsTransform = False and sits at the origin,
            # so feeding the raw world matrix to att places it on the surface
            # point (classic rivet).
            dm = node.createDecomposeMatrixNode(mtx + ".output")
            pm.connectAttr(dm + ".outputTranslate", att.attr("t"))
            pm.connectAttr(dm + ".outputRotate", att.attr("r"))

            # keep the joint scale following the global rig scale
            pm.connectAttr(dm_scl + ".outputScale", att.attr("s"))

        # Visibilities ---------------------------------
        pm.connectAttr(
            self.surfaceVis_att, self.surface.attr("visibility")
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

        # every guide locator maps to the nearest FK control / first joint
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
