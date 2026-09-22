"""Guide Match / Import - mGear Shifter helper.

Reposition the *transformable* parts of one guide's components so they match
another guide's components that share the same full name
(``comp_name + comp_side + comp_index``, e.g. ``arm_L0``).

Only the locators listed in each component guide class ``save_transform`` are
moved (this is mGear's own notion of "the parts of a guide you are allowed to
place"). ``#`` multi-locator names are expanded through the component
``minmax``. Each moved locator receives the full world matrix of the matching
reference locator - position *and* rotation. Locked / non-transform parts
(blades, size references, parameters) are never touched.

Two ways to provide the *reference* positions:

* **In-scene**    - select two guide groups. First selected = reference,
                    second selected = target (the one that gets moved).
                    i.e. "match the destination onto the source".
* **From template** - pick a ``.sgt`` / ``.json`` guide template. It is
                    imported into the scene as a new guide, then either the
                    imported guide or the existing scene guide is used as the
                    one that moves.

Components on the target that have no match on the reference are left exactly
as they are (in the template case that means "just imported, transform
unchanged").

Usage (Maya script editor / shelf button)::

    import guide_match_import
    guide_match_import.show()

Author: Fernando Jorge
"""

import os

import mgear.pymaya as pm
import maya.cmds as cmds

from mgear.vendor.Qt import QtCore, QtWidgets

from mgear.core import pyqt
from mgear import shifter
from mgear.shifter import io as shifter_io
from mgear.core import string as mg_string


# ---------------------------------------------------------------------------
# core
# ---------------------------------------------------------------------------

# status strings used in the preview table
ST_MATCH = "match  ->  will move target"
ST_NO_MATCH = "no match  ->  target unchanged"
ST_REF_ONLY = "only on reference  ->  ignored"


def _parse_guide(root_node):
    """Return a populated ``shifter.Rig().guide`` from a guide dag node.

    Args:
        root_node (pm.PyNode): guide model, or any component root under it.

    Returns:
        Guide: mGear guide object with ``.components`` / ``.componentsIndex``.
    """
    rig = shifter.Rig()
    rig.guide.setFromHierarchy(root_node, branch=True)
    return rig.guide


def _guide_model_from_selection(node):
    """Walk up to the guide model (the node with the ``ismodel`` attr)."""
    n = node
    while n is not None:
        if n.hasAttr("ismodel"):
            return n
        n = n.getParent()
    return None


def _transform_local_names(comp_guide):
    """Local names of the transformable locators of a component.

    Mirrors how ``ComponentGuide.setFromHierarchy`` walks ``save_transform``:
    ``#`` names are expanded through ``minmax`` padding, everything else is
    used verbatim. ``root`` is included - it is a transformable part.

    Args:
        comp_guide (ComponentGuide): a component guide already set from
            hierarchy.

    Returns:
        list[str]: ordered local names.
    """
    names = []
    for name in comp_guide.save_transform:
        if "#" in name:
            mm = comp_guide.minmax.get(name)
            i = 0
            while True:
                if mm is not None and mm.max > 0 and i >= mm.max:
                    break
                local = mg_string.replaceSharpWithPadding(name, i)
                node_name = comp_guide.getName(local)
                if not _node_exists(comp_guide, node_name):
                    break
                names.append(local)
                i += 1
        else:
            names.append(name)
    return names


def _resolve(comp_guide, node_name):
    """Resolve a component-local node name to a PyNode.

    Always scoped to ``comp_guide.model`` so that name clashes between two
    guides in the same scene (or a freshly imported template) resolve to the
    right guide. Returns None if not found.
    """
    from mgear.core import dag

    if comp_guide.model is not None:
        found = dag.findChild(comp_guide.model, node_name)
        if found:
            return found
    try:
        return pm.PyNode(node_name)
    except Exception:
        return None


def _node_exists(comp_guide, node_name):
    """True if ``node_name`` resolves under the guide model."""
    return _resolve(comp_guide, node_name) is not None


def build_match_plan(ref_guide, tgt_guide):
    """Compute the per-component match plan between two guides.

    Args:
        ref_guide (Guide): reference guide (positions to match).
        tgt_guide (Guide): target guide (components that will be moved).

    Returns:
        list[dict]: one row per component, keys:
            ``name``       - component full name
            ``comp_type``  - component type
            ``status``     - one of the ``ST_*`` constants
            ``locators``   - list[(tgt_node, ref_node)] pairs to snap
            ``missing``    - list[str] local names present on one side only
    """
    plan = []

    ref_names = set(ref_guide.componentsIndex)
    tgt_names = set(tgt_guide.componentsIndex)

    # target components, in guide order
    for name in tgt_guide.componentsIndex:
        tgt_comp = tgt_guide.components[name]
        row = {
            "name": name,
            "comp_type": tgt_comp.values.get("comp_type", tgt_comp.compType),
            "status": ST_NO_MATCH,
            "locators": [],
            "missing": [],
        }

        if name in ref_names:
            ref_comp = ref_guide.components[name]
            row["status"] = ST_MATCH

            ref_locals = _transform_local_names(ref_comp)
            tgt_locals = _transform_local_names(tgt_comp)
            common = [ln for ln in tgt_locals if ln in ref_locals]

            for ln in common:
                tgt_node = _resolve(tgt_comp, tgt_comp.getName(ln))
                ref_node = _resolve(ref_comp, ref_comp.getName(ln))
                if tgt_node is not None and ref_node is not None:
                    row["locators"].append((tgt_node, ref_node))

            row["missing"] = [
                ln for ln in tgt_locals if ln not in ref_locals
            ] + [ln for ln in ref_locals if ln not in tgt_locals]

        plan.append(row)

    # reference-only components (informational)
    for name in ref_guide.componentsIndex:
        if name not in tgt_names:
            ref_comp = ref_guide.components[name]
            plan.append(
                {
                    "name": name,
                    "comp_type": ref_comp.values.get("comp_type", ref_comp.compType),
                    "status": ST_REF_ONLY,
                    "locators": [],
                    "missing": [],
                }
            )

    return plan


_TR_ATTRS = (
    ("translateX", "translateY", "translateZ"),
    ("rotateX", "rotateY", "rotateZ"),
)


def _snap_tr(tgt_node, ref_node):
    """Match ``tgt_node`` world translation + rotation to ``ref_node``.

    Only unlocked translate / rotate channels on ``tgt_node`` are written, so
    locked parts of a component (e.g. an axis a component pins down) are left
    alone. Scale and shear are never touched.

    Returns:
        bool: True if any channel was written.
    """
    # any locked translate/rotate channel -> go through a temp worldspace
    # match then read back only the unlocked channels
    world_m = ref_node.getMatrix(worldSpace=True)

    # remember current unlocked-channel intent
    locked = {}
    for group in _TR_ATTRS:
        for attr in group:
            full = "{}.{}".format(tgt_node.name(), attr)
            try:
                settable = cmds.getAttr(full, settable=True)
            except Exception:
                settable = False
            locked[attr] = (not settable) or cmds.getAttr(full, lock=True)

    if not any(locked.values()):
        tgt_node.setMatrix(world_m, worldSpace=True)
        return True

    # snapshot locked channel values, do the full match, restore locked ones
    saved = {
        a: cmds.getAttr("{}.{}".format(tgt_node.name(), a))
        for a, is_l in locked.items()
        if is_l
    }
    tgt_node.setMatrix(world_m, worldSpace=True)
    wrote = False
    for a, is_l in locked.items():
        full = "{}.{}".format(tgt_node.name(), a)
        if is_l:
            was_locked = cmds.getAttr(full, lock=True)
            if was_locked:
                cmds.setAttr(full, lock=False)
            cmds.setAttr(full, saved[a])
            if was_locked:
                cmds.setAttr(full, lock=True)
        else:
            wrote = True
    return wrote


def apply_match_plan(plan):
    """Snap target locators onto their matching reference locators.

    World translation + rotation only; locked channels and scale are left
    untouched. Wrapped in a single undo chunk.

    Args:
        plan (list[dict]): output of :func:`build_match_plan`.

    Returns:
        tuple[int, int]: (components moved, locators moved).
    """
    comps_moved = 0
    locs_moved = 0

    with pm.UndoChunk():
        for row in plan:
            if row["status"] != ST_MATCH or not row["locators"]:
                continue
            moved_here = 0
            for tgt_node, ref_node in row["locators"]:
                if _snap_tr(tgt_node, ref_node):
                    moved_here += 1
            if moved_here:
                comps_moved += 1
                locs_moved += moved_here

    return comps_moved, locs_moved


def import_template(file_path):
    """Import a guide template file into the scene as a new guide.

    Args:
        file_path (str): path to a ``.sgt`` / ``.json`` guide template.

    Returns:
        pm.PyNode: the new guide model node, or None on failure.
    """
    conf = shifter_io._import_guide_template(file_path)
    if not conf:
        return None
    rig = shifter.Rig()
    rig.guide.set_from_dict(conf)
    rig.guide.draw_guide()

    # controls shapes buffer, mirrors io.import_guide_template
    if conf.get("ctl_buffers_dict"):
        from mgear.core import curve

        curve.create_curve_from_data(
            conf["ctl_buffers_dict"],
            replaceShape=True,
            rebuildHierarchy=True,
            model=rig.guide.model,
        )

    return rig.guide.model


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

class GuideMatchImportUI(QtWidgets.QDialog):

    _instance = None

    MODE_SCENE = 0
    MODE_TEMPLATE = 1

    def __init__(self, parent=None):
        super(GuideMatchImportUI, self).__init__(
            parent or pyqt.maya_main_window()
        )
        self.setObjectName("mgear_guide_match_import")
        self.setWindowTitle("Guide Match / Import")
        self.setMinimumWidth(620)
        self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

        self._ref_group = None   # pm.PyNode - reference guide root/model
        self._tgt_group = None   # pm.PyNode - target guide root/model
        self._plan = []

        self._build()
        self._connect()
        self._on_mode_changed()

    # -- construction -----------------------------------------------------

    def _build(self):
        main = QtWidgets.QVBoxLayout(self)

        # mode ----------------------------------------------------------
        mode_box = QtWidgets.QGroupBox("Reference source")
        mode_lay = QtWidgets.QHBoxLayout(mode_box)
        self.rb_scene = QtWidgets.QRadioButton("Two guides in scene")
        self.rb_template = QtWidgets.QRadioButton("Import a guide template")
        self.rb_scene.setChecked(True)
        mode_lay.addWidget(self.rb_scene)
        mode_lay.addWidget(self.rb_template)
        mode_lay.addStretch(1)
        main.addWidget(mode_box)

        # in-scene ----------------------------------------------------------
        self.scene_box = QtWidgets.QGroupBox("In-scene guides")
        sl = QtWidgets.QGridLayout(self.scene_box)

        self.ref_btn = QtWidgets.QPushButton("Set Reference  <-  selection")
        self.tgt_btn = QtWidgets.QPushButton("Set Target  <-  selection")
        self.ref_lbl = QtWidgets.QLineEdit()
        self.tgt_lbl = QtWidgets.QLineEdit()
        for le in (self.ref_lbl, self.tgt_lbl):
            le.setReadOnly(True)
        self.swap_btn = QtWidgets.QPushButton("Swap")
        self.pick_both_btn = QtWidgets.QPushButton(
            "Use current selection  (1st = reference, 2nd = target)"
        )

        sl.addWidget(QtWidgets.QLabel("Reference (not moved):"), 0, 0)
        sl.addWidget(self.ref_lbl, 0, 1)
        sl.addWidget(self.ref_btn, 0, 2)
        sl.addWidget(QtWidgets.QLabel("Target (gets moved):"), 1, 0)
        sl.addWidget(self.tgt_lbl, 1, 1)
        sl.addWidget(self.tgt_btn, 1, 2)
        sl.addWidget(self.swap_btn, 2, 2)
        sl.addWidget(self.pick_both_btn, 3, 0, 1, 3)
        main.addWidget(self.scene_box)

        # template ----------------------------------------------------------
        self.tpl_box = QtWidgets.QGroupBox("Guide template")
        tl = QtWidgets.QGridLayout(self.tpl_box)
        self.tpl_path = QtWidgets.QLineEdit()
        self.tpl_browse = QtWidgets.QPushButton("...")
        self.tpl_browse.setMaximumWidth(36)
        tl.addWidget(QtWidgets.QLabel("Template file:"), 0, 0)
        tl.addWidget(self.tpl_path, 0, 1)
        tl.addWidget(self.tpl_browse, 0, 2)

        self.rb_tpl_is_target = QtWidgets.QRadioButton(
            "Move the IMPORTED guide onto the scene guide"
        )
        self.rb_tpl_is_ref = QtWidgets.QRadioButton(
            "Move the SCENE guide onto the imported guide"
        )
        self.rb_tpl_is_target.setChecked(True)
        tl.addWidget(self.rb_tpl_is_target, 1, 0, 1, 3)
        tl.addWidget(self.rb_tpl_is_ref, 2, 0, 1, 3)

        self.scene_guide_btn = QtWidgets.QPushButton(
            "Set scene guide  <-  selection"
        )
        self.scene_guide_lbl = QtWidgets.QLineEdit()
        self.scene_guide_lbl.setReadOnly(True)
        tl.addWidget(QtWidgets.QLabel("Scene guide:"), 3, 0)
        tl.addWidget(self.scene_guide_lbl, 3, 1)
        tl.addWidget(self.scene_guide_btn, 3, 2)
        main.addWidget(self.tpl_box)

        # preview ----------------------------------------------------------
        self.preview_btn = QtWidgets.QPushButton("Preview matches")
        main.addWidget(self.preview_btn)

        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(
            ["Component", "Type", "Match status", "Locators moved"]
        )
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setColumnWidth(0, 200)
        self.table.setColumnWidth(1, 150)
        self.table.setColumnWidth(2, 170)
        main.addWidget(self.table)

        self.summary_lbl = QtWidgets.QLabel("")
        main.addWidget(self.summary_lbl)

        # apply ----------------------------------------------------------
        self.apply_btn = QtWidgets.QPushButton("Apply")
        self.apply_btn.setStyleSheet("font-weight: bold; padding: 6px;")
        main.addWidget(self.apply_btn)

    def _connect(self):
        self.rb_scene.toggled.connect(self._on_mode_changed)
        self.ref_btn.clicked.connect(lambda: self._pick_group("ref"))
        self.tgt_btn.clicked.connect(lambda: self._pick_group("tgt"))
        self.scene_guide_btn.clicked.connect(lambda: self._pick_group("ref"))
        self.swap_btn.clicked.connect(self._swap)
        self.pick_both_btn.clicked.connect(self._pick_both)
        self.tpl_browse.clicked.connect(self._browse_template)
        self.preview_btn.clicked.connect(self.preview)
        self.apply_btn.clicked.connect(self.apply)

    # -- helpers -----------------------------------------------------

    def _mode(self):
        return self.MODE_SCENE if self.rb_scene.isChecked() else self.MODE_TEMPLATE

    def _on_mode_changed(self, *args):
        scene = self._mode() == self.MODE_SCENE
        self.scene_box.setVisible(scene)
        self.tpl_box.setVisible(not scene)
        self.adjustSize()

    @staticmethod
    def _selected_guide_root():
        sel = pm.selected(type="transform")
        if not sel:
            return None
        node = sel[0]
        model = _guide_model_from_selection(node)
        return model or node

    def _pick_group(self, which):
        root = self._selected_guide_root()
        if not root:
            pm.displayWarning("Select a guide group (or a node under it).")
            return
        if which == "ref":
            self._ref_group = root
            self.ref_lbl.setText(root.name())
            self.scene_guide_lbl.setText(root.name())
        else:
            self._tgt_group = root
            self.tgt_lbl.setText(root.name())

    def _pick_both(self):
        sel = pm.selected(type="transform")
        if len(sel) < 2:
            pm.displayWarning(
                "Select two guide groups: first = reference, second = target."
            )
            return
        self._ref_group = _guide_model_from_selection(sel[0]) or sel[0]
        self._tgt_group = _guide_model_from_selection(sel[1]) or sel[1]
        self.ref_lbl.setText(self._ref_group.name())
        self.tgt_lbl.setText(self._tgt_group.name())

    def _swap(self):
        self._ref_group, self._tgt_group = self._tgt_group, self._ref_group
        self.ref_lbl.setText(self._ref_group.name() if self._ref_group else "")
        self.tgt_lbl.setText(self._tgt_group.name() if self._tgt_group else "")

    def _browse_template(self):
        start = self.tpl_path.text() or cmds.workspace(q=True, rd=True)
        fp, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Guide template",
            start,
            "Guide template (*.sgt *.scg *.json);;All files (*.*)",
        )
        if fp:
            self.tpl_path.setText(fp)

    # -- resolve the two guides for the current mode ------------------------

    def _resolve_guides(self, do_import):
        """Return (ref_guide, tgt_guide, note) or (None, None, error).

        Args:
            do_import (bool): if True and in template mode, actually import
                the template. If False, template mode cannot resolve and
                returns an error asking to Apply.
        """
        if self._mode() == self.MODE_SCENE:
            if not self._ref_group or not self._tgt_group:
                return None, None, "Set both a reference and a target guide."
            if self._ref_group == self._tgt_group:
                return None, None, "Reference and target are the same guide."
            ref = _parse_guide(self._ref_group)
            tgt = _parse_guide(self._tgt_group)
            return ref, tgt, ""

        # template mode
        path = self.tpl_path.text().strip()
        if not path or not os.path.isfile(path):
            return None, None, "Pick a valid guide template file."
        if not self._ref_group:
            return None, None, "Set the scene guide."
        scene_guide_node = self._ref_group

        if not do_import:
            return None, None, (
                "Template will be imported on Apply. "
                "Press Apply to import and match."
            )

        new_model = import_template(path)
        if not new_model:
            return None, None, "Failed to import the template."

        scene_guide = _parse_guide(scene_guide_node)
        imported_guide = _parse_guide(new_model)

        if self.rb_tpl_is_target.isChecked():
            # imported guide moves onto the scene guide
            return scene_guide, imported_guide, (
                "Imported '%s'. Moving it onto '%s'."
                % (new_model.name(), scene_guide_node.name())
            )
        else:
            # scene guide moves onto the imported guide
            return imported_guide, scene_guide, (
                "Imported '%s'. Moving '%s' onto it."
                % (new_model.name(), scene_guide_node.name())
            )

    # -- actions -----------------------------------------------------

    def _fill_table(self, plan):
        self.table.setRowCount(0)
        colors = {
            ST_MATCH: QtCore.Qt.green,
            ST_NO_MATCH: QtCore.Qt.yellow,
            ST_REF_ONLY: QtCore.Qt.gray,
        }
        for row in plan:
            r = self.table.rowCount()
            self.table.insertRow(r)
            items = [
                QtWidgets.QTableWidgetItem(row["name"]),
                QtWidgets.QTableWidgetItem(row["comp_type"]),
                QtWidgets.QTableWidgetItem(row["status"]),
                QtWidgets.QTableWidgetItem(str(len(row["locators"]))),
            ]
            if row["missing"]:
                items[2].setToolTip(
                    "unmatched locators: " + ", ".join(sorted(set(row["missing"])))
                )
            for c, it in enumerate(items):
                if c == 2:
                    it.setForeground(colors.get(row["status"], QtCore.Qt.white))
                self.table.setItem(r, c, it)

        n_match = sum(1 for x in plan if x["status"] == ST_MATCH)
        n_nomatch = sum(1 for x in plan if x["status"] == ST_NO_MATCH)
        n_refonly = sum(1 for x in plan if x["status"] == ST_REF_ONLY)
        n_locs = sum(len(x["locators"]) for x in plan)
        self.summary_lbl.setText(
            "%d matched (%d locators), %d target-only (unchanged), "
            "%d reference-only (ignored)"
            % (n_match, n_locs, n_nomatch, n_refonly)
        )

    def preview(self):
        ref, tgt, note = self._resolve_guides(do_import=False)
        if ref is None:
            self.summary_lbl.setText(note)
            self.table.setRowCount(0)
            self._plan = []
            return
        self._plan = build_match_plan(ref, tgt)
        self._fill_table(self._plan)
        if note:
            self.summary_lbl.setText(note + "  " + self.summary_lbl.text())

    def apply(self):
        ref, tgt, note = self._resolve_guides(do_import=True)
        if ref is None:
            QtWidgets.QMessageBox.warning(self, "Guide Match / Import", note)
            return
        plan = build_match_plan(ref, tgt)
        self._plan = plan
        self._fill_table(plan)

        comps, locs = apply_match_plan(plan)
        msg = "%s\nMoved %d components (%d locators)." % (note or "Done.", comps, locs)
        pm.displayInfo(msg.replace("\n", " "))
        self.summary_lbl.setText(msg.splitlines()[-1] + "  " + self.summary_lbl.text())


# ---------------------------------------------------------------------------

def show(*args):
    """Show the Guide Match / Import window."""
    parent = pyqt.maya_main_window()
    for child in parent.children():
        if (
            isinstance(child, QtWidgets.QDialog)
            and child.objectName() == "mgear_guide_match_import"
        ):
            child.close()
            child.deleteLater()
    win = GuideMatchImportUI()
    win.show()
    return win


if __name__ == "__main__":
    show()
