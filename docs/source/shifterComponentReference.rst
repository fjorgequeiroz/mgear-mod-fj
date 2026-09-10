.. _shifter-component-reference:

Shifter Component Reference
###########################

Shifter comes with over 40 components you can build your rig from. This section covers the functionality and settings of each one in detail.

Guide Settings
==============

Main Settings
=============

Settings shared by all components.

* **Name**: The base name of the component, all the parts of the component get renmaed.
* **Side**: Which side the component is on; Left, Right or Center
* **Component Index**: WIP
* **Connector**: WIP

Joint Connection Settings
-------------------------

* **Use Joint Index**: WIP
* **Parent Joint Index**: WIP

Channel Host Settings
---------------------

* **Host**: The guide that stores the extra attributes for this component, such as IK/FK blendering and more. This will generally be a control_01 component.

Custom Ctonrollers Group
------------------------

WIP

----------------------------------------------------------------------------------------------------------------

Shifter Components
==================

arm_2jnt_01
-----------

Two bone arm setup with:

* **IK/FK blending** for switching between animating with IK or FK
* **Roll** attribute that lets you adjust the elbow direction, without moving the pole vector
* **Twist joints** for better deformations around the shoulder and wrist joints
* **Armpit roll** to manually adjust the twist joints around the shoulder joint
* **Scale** of the entire arm
* **Stretching** so the arm can elongate when the IK control is too far away to reach, including a maximum stretch limit
* **Sliding** of the elbow, which elongates the upper arm while shortening the lower arm, or vice versa
* **Softness** to prevent popping when the IK arm is stretched out straight
* **Reverse** to make the arm bend in the opposite direction when using IK
* **Roundness** for rubber hose style animation
* **Volume** preservation during stretching
* **Elbow** controller for accurately positioning the elbow, for instance when a character is resting her elbows on a table
* **Space switchers** for the IK, Up Vector and Elbow controllers that controls which space they follow

This is the default arm comoponent used by the biped template rig.

Guide Positioning
+++++++++++++++++

WIP

Compoent Settings
+++++++++++++++++

TODO! Should this be tools tips in the interface itself.

* **IK/FK Blender**: Default 

Source
++++++

Link to Module (how)

arm_2jnt_02
-----------
WIP

arm_2jnt_03
-----------
WIP

arm_2jnt_04
-----------
WIP

arm_2jnt_freeTangents_01
------------------------
WIP

arm_ms_2jnt_01
--------------
WIP

cable_01
--------
WIP

chain_01
--------
WIP

chain_FK_spline_01
------------------
WIP

chain_FK_spline_02
------------------
WIP

chain_FK_spline_variable_IK_01
------------------------------
WIP

chain_IK_spline_variable_FK_01
------------------------------
WIP

chain_IK_spline_variable_FK_stack_01
------------------------------------
WIP

chain_net_01
------------
WIP

chain_spring_01
---------------
WIP

chain_stack_01
--------------
WIP

chain_whip_01
-------------
WIP

control_01
----------
WIP

eye_01
------
WIP

foot_bk_01
----------
WIP

hydraulic_01
------------
WIP

leg_2jnt_01
-----------
WIP

leg_2jnt_02
-----------
WIP

leg_2jnt_freeTangents_01
------------------------
WIP

leg_3jnt_01
-----------
WIP

leg_ms_2jnt_01
--------------
WIP

lite_chain_01
-------------
WIP

lite_chain_stack_01
-------------------
WIP

meta_01
-------
WIP

mouth_01
--------
WIP

mouth_02
--------
WIP

neck_fk_01
----------

FK neck. This is the FK-primary counterpart of ``neck_ik_01``: the dependency
direction is reversed so that the FK controllers drive the IK controller
instead of the other way around.

**Behaviour**

* The ``fk0_ctl`` ... ``fkN_ctl`` chain is a real, parented FK hierarchy and
  is the only rig input. Rotating a control rotates every control below it,
  exactly like a plain FK chain.
* The ``ik_ctl`` (compas shape, at the tip of the neck) is kept for
  familiarity and for anything downstream that expects a neck "ik" handle,
  but it is a **locked, non-keyable read-out**: it is matrix-driven by the
  last FK control, so it always sits at the tip of the neck and follows the
  chain. It cannot be animated.
* The ``head_ctl`` is parented under the last FK control and behaves the same
  as in ``neck_ik_01`` (including its Head Reference Array space switch).
* Optional per-segment squash and stretch, driven by the ``Volume`` attribute
  and the Squash and Stretch profile. The chain length is measured live from
  the FK controls, so translating a control still triggers volume
  preservation.

**Why there is no cycle**

``neck_ik_01`` makes the IK control the master through an IK spline: a master
curve, a ``gear_curveslide2`` solver, a slave curve and path-constrained
``div_cns`` transforms, plus a ``gear_intmatrix_op`` roll that reads
``ik_ctl.worldMatrix``. ``neck_fk_01`` removes all of that. The only link
between the FK chain and the ``ik_ctl`` is
``fk_ctl[-1].worldMatrix -> ik_cns`` and it is one-directional. Because
``ik_ctl`` is fully locked, no animator, parent or reference array can feed a
transform back into the FK chain, so a dependency cycle is impossible by
construction.

**Settings**

* **Divisions**: Number of FK controls / deform joints along the neck
  (minimum 3).
* **Tangent Controls**: Adds two cosmetic tangent controls used only for the
  display curve.
* **IK Ctl World Ori**: Aligns the locked IK read-out control to world space.
* **Squash and Stretch Profile**: FCurve profile for the optional volume
  preservation.
* **FK Base Reference Array**: Space switch for the base of the neck (drives
  ``fk0_npo``, above every control, so it never touches the IK read-out).
* **Head Reference Array**: Space switch for the head control.

**Guide locators**: ``root``, ``tan0``, ``tan1``, ``neck``, ``head``,
``eff`` (identical to ``neck_ik_01``, so an existing neck guide can be
retyped).

neck_ik_01
----------
WIP

sdk_control_01
--------------
WIP

shoulder_01
-----------
WIP

shoulder_02
-----------
WIP

shoulder_ms_01
--------------
WIP

spine_FK_01
-----------
WIP

spine_S_shape_01
----------------
WIP

spine_ik_01
-----------
WIP

spine_ik_02
-----------
WIP

squash4Sides_01
---------------
WIP

squash_01
---------
WIP

tangent_spline_01
-----------------
WIP

ui_container_01
---------------
WIP

ui_slider_01
------------
WIP
