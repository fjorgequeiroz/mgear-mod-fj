"""Guide Match / Import - mGear Shifter helper.

Reposition the *transformable* parts of one guide's components so they match
another guide's components that share the same full name
(``comp_name + comp_side + comp_index``, e.g. ``arm_L0``).

Only the locators listed in each component guide class ``save_transform`` are
moved (this is mGear's own notion of "the parts of a guide you are allowed to
place"). ``#`` multi-locator names are expanded through the component
``minmax``. Each moved locator is FORCED onto the matching reference
locator's world matrix - translation *and* rotation - unlocking a locked
channel just long enough to write it and re-locking it after, so a locked
channel never silently blocks the match. Scale / shear and non-transform
parts (blades, size references, parameters) are never touched.

Two ways to provide the *reference* positions:

* **In-scene**    - select two guide groups. First selected = reference,
                    second selected = target (the one that gets moved).
                    i.e. "match the destination onto the source".
* **From template** - pick a ``.sgt`` / ``.json`` guide template. It is
                    imported into the scene as a new guide, then either the
                    imported guide or the existing scene guide is used as the
                    one that moves.

Those two pickers set the *default* reference/target for every row, but the
preview table's Source guide / Target guide columns are editable per row:
each is a dropdown of every guide currently in the scene (refreshed with the
"Refresh guide list" button, or automatically after a template import), so
any single component can be re-pointed at a different guide than the rest of
the table without redoing the whole match. Changing a row's dropdown
re-resolves just that row immediately.

Components on the target that have no match on the reference are left exactly
as they are (in the template case that means "just imported, transform
unchanged").

Usage (Maya script editor / shelf button)::

    import guide_match_import
    guide_match_import.show()

While iterating on this file (pulling updates, editing it), Python's module
cache means a plain re-``import`` + ``show()`` keeps running the OLD code
already loaded in this Maya session - symptoms include fixes appearing to
"not work" and UI changes (like sortable columns) not showing up. Force a
reload first::

    import importlib
    import guide_match_import
    importlib.reload(guide_match_import)
    guide_match_import.show()

(Python 2 / older Maya: ``reload(guide_match_import)`` instead of the
``importlib`` call.) Or simplest of all: just restart Maya.

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
ST_NOT_CRAWLED = "NOT FOUND by guide crawl  ->  check hierarchy / comp module"


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


def scan_component_roots(model_node):
    """Ground-truth list of every ``comp_type`` node under a guide model.

    Independent of ``Guide.setFromHierarchy`` / ``findComponentRecursive`` -
    a plain ``cmds.ls`` + attribute check. Used to detect components that
    mGear's own recursive crawl silently drops (e.g. a component root that
    isn't reachable by walking ``transform`` children only, or one whose
    component module failed to import).

    Args:
        model_node (pm.PyNode): the guide model (``ismodel`` node).

    Returns:
        set[str]: full names (``comp_name_compSide+compIndex``) found.
    """
    found = set()
    top = model_node.longName() if hasattr(model_node, "longName") else model_node.name()
    descendants = cmds.listRelatives(
        top, allDescendents=True, fullPath=True, type="transform"
    ) or []
    for node in descendants:
        if cmds.attributeQuery("comp_type", node=node, exists=True):
            try:
                cname = cmds.getAttr(node + ".comp_name")
                cside = cmds.getAttr(node + ".comp_side")
                cidx = cmds.getAttr(node + ".comp_index")
                found.add("{}_{}{}".format(cname, cside, cidx))
            except Exception:
                pass
    return found


def diagnose_guide(guide_obj, model_node):
    """Compare mGear's crawled component list against a raw scene scan.

    Args:
        guide_obj (Guide): result of :func:`_parse_guide`.
        model_node (pm.PyNode): the guide model that was parsed.

    Returns:
        dict: ``valid`` (bool, mirrors ``guide.valid``), ``crawled`` (set),
            ``scanned`` (set), ``missed`` (scanned - crawled, i.e. components
            that exist in the scene but the crawl did not pick up).
    """
    crawled = set(guide_obj.componentsIndex)
    scanned = scan_component_roots(model_node)
    return {
        "valid": guide_obj.valid,
        "crawled": crawled,
        "scanned": scanned,
        "missed": scanned - crawled,
    }


def _guide_model_from_selection(node):
    """Walk up to the guide model (the node with the ``ismodel`` attr)."""
    n = node
    while n is not None:
        if n.hasAttr("ismodel"):
            return n
        n = n.getParent()
    return None


def list_guide_models():
    """Every guide model (``ismodel`` node) currently in the scene.

    Returns:
        list[pm.PyNode]: guide model transforms, alphabetically by name.
    """
    names = cmds.ls("*", type="transform", long=True) or []
    models = []
    for n in names:
        if cmds.attributeQuery("ismodel", node=n, exists=True):
            try:
                models.append(pm.PyNode(n))
            except Exception:
                pass
    return sorted(models, key=lambda m: m.name())


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


def match_single_component(ref_guide, tgt_guide, name):
    """Build the match-plan row for one component full name.

    Shared by :func:`build_match_plan` (bulk, all target components) and the
    UI's per-row Source/Target override (re-resolve one row against a guide
    picked just for that row).

    Args:
        ref_guide (Guide): reference guide (positions to match), or None.
        tgt_guide (Guide): target guide (component that gets moved), or None.
        name (str): component full name to look up on both sides.

    Returns:
        dict: a plan row, see :func:`build_match_plan`.
    """
    tgt_comp = tgt_guide.components.get(name) if tgt_guide else None
    ref_comp = ref_guide.components.get(name) if ref_guide else None

    if tgt_comp is None and ref_comp is None:
        return {
            "name": name,
            "comp_type": "?",
            "status": ST_NOT_CRAWLED,
            "locators": [],
            "missing": [],
        }

    if tgt_comp is None:
        # only exists on the reference side
        return {
            "name": name,
            "comp_type": ref_comp.values.get("comp_type", ref_comp.compType),
            "status": ST_REF_ONLY,
            "locators": [],
            "missing": [],
        }

    row = {
        "name": name,
        "comp_type": tgt_comp.values.get("comp_type", tgt_comp.compType),
        "status": ST_NO_MATCH,
        "locators": [],
        "missing": [],
    }

    if ref_comp is not None:
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

    return row


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
        plan.append(match_single_component(ref_guide, tgt_guide, name))

    # reference-only components (informational)
    for name in ref_guide.componentsIndex:
        if name not in tgt_names:
            plan.append(match_single_component(ref_guide, None, name))

    return plan


_TR_ATTRS = (
    ("translateX", "translateY", "translateZ"),
    ("rotateX", "rotateY", "rotateZ"),
)


def _snap_tr(tgt_node, ref_node):
    """Force ``tgt_node``'s world translation + rotation onto ``ref_node``'s.

    The full world matrix (translation + rotation) is always written,
    unlocking any locked translate/rotate channel just long enough to set it
    and re-locking it after, so the position match is never silently
    dropped by a locked channel. Scale and shear are never touched.

    Returns:
        bool: True (kept for call-site compatibility / future use).
    """
    world_m = ref_node.getMatrix(worldSpace=True)

    # Use the full (long) dag path for every cmds.* call below. ``.name()``
    # returns Maya's *partial* path (the shortest string that currently
    # disambiguates the node), which can silently point at the wrong node
    # once two guides - or a nested vs. top-level copy of the same locator
    # name - are loaded in the same scene. Full paths are always unambiguous.
    tgt_path = tgt_node.longName()

    locked_attrs = []
    for group in _TR_ATTRS:
        for attr in group:
            full = "{}.{}".format(tgt_path, attr)
            if cmds.getAttr(full, lock=True):
                locked_attrs.append(full)
                cmds.setAttr(full, lock=False)

    try:
        tgt_node.setMatrix(world_m, worldSpace=True)
    finally:
        for full in locked_attrs:
            cmds.setAttr(full, lock=True)

    return True


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
        self.setMinimumWidth(900)
        self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

        self._ref_group = None   # pm.PyNode - reference guide root/model
        self._tgt_group = None   # pm.PyNode - target guide root/model
        self._ref_model_node = None  # resolved model used by the last run
        self._tgt_model_node = None
        self._plan = {}  # {component full name: plan row dict}

        # Source/Target column support: every guide model currently in the
        # scene, and a lazily-populated cache of their parsed Guide objects
        # (parsing walks the whole hierarchy, so it is not re-done per row).
        self._guide_choices = []       # list[pm.PyNode] model nodes
        self._guide_cache = {}         # model node name (str) -> Guide obj

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

        # Column(s): Component | Source guide | Target guide | Type |
        #            Match status | Locators moved
        self.COL_NAME, self.COL_SRC, self.COL_TGT, self.COL_TYPE, \
            self.COL_STATUS, self.COL_LOCS = range(6)

        self.table = QtWidgets.QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            [
                "Component",
                "Source guide",
                "Target guide",
                "Type",
                "Match status",
                "Locators moved",
            ]
        )
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionsClickable(True)
        self.table.setSortingEnabled(True)
        self.table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setColumnWidth(self.COL_NAME, 170)
        self.table.setColumnWidth(self.COL_SRC, 150)
        self.table.setColumnWidth(self.COL_TGT, 150)
        self.table.setColumnWidth(self.COL_TYPE, 130)
        self.table.setColumnWidth(self.COL_STATUS, 170)
        main.addWidget(self.table)

        refresh_row = QtWidgets.QHBoxLayout()
        self.refresh_guides_btn = QtWidgets.QPushButton(
            "Refresh guide list (Source/Target columns)"
        )
        refresh_row.addWidget(self.refresh_guides_btn)
        refresh_row.addStretch(1)
        main.addLayout(refresh_row)

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
        self.refresh_guides_btn.clicked.connect(self._refresh_guide_choices)

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

    # -- per-row Source/Target guide picking -----------------------------

    def _guide_for(self, model_node):
        """Parsed ``Guide`` for a model node, using/populating the cache.

        Args:
            model_node (pm.PyNode): a guide model (``ismodel`` node).

        Returns:
            Guide: parsed guide object.
        """
        key = model_node.name(long=True)
        guide_obj = self._guide_cache.get(key)
        if guide_obj is None:
            guide_obj = _parse_guide(model_node)
            self._guide_cache[key] = guide_obj
        return guide_obj

    def _refresh_guide_choices(self):
        """Re-scan the scene for guide models and rebuild every row's combo.

        Clears the parsed-guide cache too, so edits made to a guide since
        the last scan (e.g. after an Apply) are picked up.
        """
        self._guide_cache = {}
        self._guide_choices = list_guide_models()
        for r in range(self.table.rowCount()):
            for col in (self.COL_SRC, self.COL_TGT):
                combo = self.table.cellWidget(r, col)
                if combo is not None:
                    self._populate_row_combo(combo)

    def _populate_row_combo(self, combo):
        """Fill a row's guide combobox, keeping its current pick if still valid."""
        current = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        for model_node in self._guide_choices:
            combo.addItem(model_node.name(), model_node)
        combo.blockSignals(False)
        if current is not None:
            for i in range(combo.count()):
                if combo.itemData(i) == current:
                    combo.setCurrentIndex(i)
                    break

    def _make_row_combo(self, component_name, preselect):
        """Build a Source/Target guide combobox for one table row.

        The component full name is stamped on the widget itself (Qt dynamic
        property) rather than relied on via row index, because
        ``setSortingEnabled(True)`` lets the user reorder table rows by
        clicking a header - a captured row index would go stale the moment
        that happens. Looking the row up by the widget's own identity keeps
        this correct no matter how the table is currently sorted.

        Args:
            component_name (str): full name this combo belongs to.
            preselect (pm.PyNode or None): guide model to select initially.

        Returns:
            QComboBox
        """
        combo = QtWidgets.QComboBox()
        combo.setEditable(False)
        combo.setProperty("component_name", component_name)
        for model_node in self._guide_choices:
            combo.addItem(model_node.name(), model_node)
        if preselect is not None:
            for i in range(combo.count()):
                if combo.itemData(i) == preselect:
                    combo.setCurrentIndex(i)
                    break
        combo.currentIndexChanged.connect(self._on_row_combo_changed)
        return combo

    def _find_row_by_name(self, name):
        """Current table row index displaying component ``name``, or -1.

        Looked up by the Source combo's stamped ``component_name`` property
        rather than list position, so it stays correct after the user sorts
        the table by clicking a header.
        """
        for r in range(self.table.rowCount()):
            w = self.table.cellWidget(r, self.COL_SRC)
            if w is not None and w.property("component_name") == name:
                return r
        return -1

    def _on_row_combo_changed(self, *_args):
        combo = self.sender()
        if combo is None:
            return
        name = combo.property("component_name")
        if name:
            self._rebuild_row(name)

    def _rebuild_row(self, name):
        """Re-resolve a single component from its own combo picks.

        Args:
            name (str): component full name - the stable key. The table row
                that currently displays it is found by widget identity, so
                this is correct regardless of the table's current sort
                order.
        """
        r = self._find_row_by_name(name)
        if r < 0:
            return
        src_combo = self.table.cellWidget(r, self.COL_SRC)
        tgt_combo = self.table.cellWidget(r, self.COL_TGT)
        src_model = src_combo.currentData() if src_combo else None
        tgt_model = tgt_combo.currentData() if tgt_combo else None

        ref_guide = self._guide_for(src_model) if src_model is not None else None
        tgt_guide = self._guide_for(tgt_model) if tgt_model is not None else None

        row = match_single_component(ref_guide, tgt_guide, name)
        self._plan[name] = row
        self._paint_row(r, row)
        self._update_summary()

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
            ref = self._guide_for(self._ref_group)
            tgt = self._guide_for(self._tgt_group)
            self._ref_model_node = self._ref_group
            self._tgt_model_node = self._tgt_group
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

        # a brand new model just appeared in the scene - make it (and any
        # other new guide) available in the per-row Source/Target combos
        self._guide_choices = list_guide_models()

        scene_guide = self._guide_for(scene_guide_node)
        imported_guide = self._guide_for(new_model)

        if self.rb_tpl_is_target.isChecked():
            # imported guide moves onto the scene guide
            self._ref_model_node = scene_guide_node
            self._tgt_model_node = new_model
            return scene_guide, imported_guide, (
                "Imported '%s'. Moving it onto '%s'."
                % (new_model.name(), scene_guide_node.name())
            )
        else:
            # scene guide moves onto the imported guide
            self._ref_model_node = new_model
            self._tgt_model_node = scene_guide_node
            return imported_guide, scene_guide, (
                "Imported '%s'. Moving '%s' onto it."
                % (new_model.name(), scene_guide_node.name())
            )

    # -- actions -----------------------------------------------------

    def _append_crawl_diagnostics(self, plan, ref, tgt):
        """Append rows for components the crawl missed on either side.

        Compares ``ref``/``tgt`` ``componentsIndex`` against a raw scene
        scan (``scan_component_roots``) using the model nodes recorded by
        the last ``_resolve_guides`` call. Mutates ``plan`` in place and
        returns the combined set of missed full names, for the summary.
        """
        missed_names = set()
        for guide_obj, model_node, side in (
            (ref, self._ref_model_node, "reference"),
            (tgt, self._tgt_model_node, "target"),
        ):
            if model_node is None:
                continue
            diag = diagnose_guide(guide_obj, model_node)
            for name in sorted(diag["missed"]):
                missed_names.add(name)
                plan.append(
                    {
                        "name": name,
                        "comp_type": "?",
                        "status": ST_NOT_CRAWLED,
                        "locators": [],
                        "missing": [],
                        "_side": side,
                    }
                )
        return missed_names

    _STATUS_COLORS = {
        ST_MATCH: QtCore.Qt.green,
        ST_NO_MATCH: QtCore.Qt.yellow,
        ST_REF_ONLY: QtCore.Qt.gray,
        ST_NOT_CRAWLED: QtCore.Qt.red,
    }

    def _paint_row(self, r, row):
        """(Re)write the Component / Type / Match status / Locators moved
        cells of row ``r`` from a plan row dict. Does not touch the
        Source/Target combo cells - those are the input, not the output.
        """
        n_locs_item = QtWidgets.QTableWidgetItem()
        n_locs_item.setData(QtCore.Qt.DisplayRole, len(row["locators"]))

        name_item = QtWidgets.QTableWidgetItem(row["name"])
        type_item = QtWidgets.QTableWidgetItem(row["comp_type"])
        status_item = QtWidgets.QTableWidgetItem(row["status"])

        if row["missing"]:
            status_item.setToolTip(
                "unmatched locators: " + ", ".join(sorted(set(row["missing"])))
            )
        if row["status"] == ST_NOT_CRAWLED:
            status_item.setToolTip(
                "Exists in the %s scene hierarchy (comp_type attr found)"
                " but Guide.setFromHierarchy did not pick it up - it is"
                " skipped by the matcher entirely. Common causes: it is"
                " parented under something other than a plain transform"
                " chain, its component module failed to import (check"
                " Script Editor), or comp_name/comp_side/comp_index"
                " differs from what it looks like in the Outliner."
                % row.get("_side", "?")
            )
        status_item.setForeground(
            self._STATUS_COLORS.get(row["status"], QtCore.Qt.white)
        )

        self.table.setItem(r, self.COL_NAME, name_item)
        self.table.setItem(r, self.COL_TYPE, type_item)
        self.table.setItem(r, self.COL_STATUS, status_item)
        self.table.setItem(r, self.COL_LOCS, n_locs_item)

    def _fill_table(self, plan_rows):
        """(Re)build the whole table from a list of plan rows.

        Every row defaults its Source/Target combos to the currently
        resolved reference/target guide (``self._ref_model_node`` /
        ``self._tgt_model_node``); override any row afterwards via its own
        combo. Rebuilds ``self._plan`` as a ``{name: row}`` dict - table
        rows are looked up by the combo's stamped component name rather
        than by list/row position, so re-sorting the table by clicking a
        header never desyncs a row from its data.
        """
        self.table.setSortingEnabled(False)
        self.table.setRowCount(0)
        self._plan = {}

        for row in plan_rows:
            name = row["name"]
            self._plan[name] = row
            r = self.table.rowCount()
            self.table.insertRow(r)

            src_combo = self._make_row_combo(name, self._ref_model_node)
            tgt_combo = self._make_row_combo(name, self._tgt_model_node)
            self.table.setCellWidget(r, self.COL_SRC, src_combo)
            self.table.setCellWidget(r, self.COL_TGT, tgt_combo)

            self._paint_row(r, row)

        self.table.setSortingEnabled(True)
        self._update_summary()

    def _update_summary(self):
        rows = list(self._plan.values())
        n_match = sum(1 for x in rows if x["status"] == ST_MATCH)
        n_nomatch = sum(1 for x in rows if x["status"] == ST_NO_MATCH)
        n_refonly = sum(1 for x in rows if x["status"] == ST_REF_ONLY)
        n_missed = sum(1 for x in rows if x["status"] == ST_NOT_CRAWLED)
        n_locs = sum(len(x["locators"]) for x in rows)
        summary = (
            "%d matched (%d locators), %d target-only (unchanged), "
            "%d reference-only (ignored)"
            % (n_match, n_locs, n_nomatch, n_refonly)
        )
        if n_missed:
            summary += "  |  %d NOT FOUND by crawl (see red rows)" % n_missed
        self.summary_lbl.setText(summary)

    def preview(self):
        self._guide_choices = list_guide_models()
        ref, tgt, note = self._resolve_guides(do_import=False)
        if ref is None:
            self.summary_lbl.setText(note)
            self.table.setRowCount(0)
            self._plan = {}
            return
        plan_rows = build_match_plan(ref, tgt)
        self._append_crawl_diagnostics(plan_rows, ref, tgt)
        self._fill_table(plan_rows)
        if note:
            self.summary_lbl.setText(note + "  " + self.summary_lbl.text())

    def apply(self):
        note = ""

        if self._mode() == self.MODE_TEMPLATE:
            # Template mode can't be meaningfully previewed beforehand (the
            # source guide does not exist until it is imported), so Apply
            # does the import + a fresh full match itself, same as before
            # the per-row Source/Target columns existed. The result still
            # populates the table with per-row combos afterwards so it can
            # be reviewed / hand-edited if the user wants to Apply again.
            ref, tgt, note = self._resolve_guides(do_import=True)
            if ref is None:
                QtWidgets.QMessageBox.warning(self, "Guide Match / Import", note)
                return
            self._guide_choices = list_guide_models()
            plan_rows = build_match_plan(ref, tgt)
            self._append_crawl_diagnostics(plan_rows, ref, tgt)
            self._fill_table(plan_rows)
        else:
            # Scene mode: Apply always uses what is currently sitting in
            # each row's own Source/Target combo (built during the last
            # Preview, and editable per row since) - that is the whole
            # point of the per-row override, so a Preview must exist first.
            if not self._plan or self.table.rowCount() != len(self._plan):
                QtWidgets.QMessageBox.warning(
                    self,
                    "Guide Match / Import",
                    "Run Preview first (and use the Source/Target columns "
                    "to override individual rows if needed) before Apply.",
                )
                return
            # re-resolve every row from its own (possibly hand-picked)
            # combo, so Apply reflects any edits made after Preview.
            # Rebuild by name, not row index - the table may be sorted.
            for name in list(self._plan.keys()):
                self._rebuild_row(name)

        rows = list(self._plan.values())
        missed = [row["name"] for row in rows if row["status"] == ST_NOT_CRAWLED]

        if missed:
            proceed = QtWidgets.QMessageBox.warning(
                self,
                "Guide Match / Import",
                "%d component(s) exist in the scene but were NOT picked up "
                "by the guide crawl, so they will be skipped:\n\n%s\n\n"
                "Proceed with the components that were found?"
                % (len(missed), "\n".join(sorted(missed))),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
            )
            if proceed != QtWidgets.QMessageBox.Yes:
                return

        comps, locs = apply_match_plan(rows)
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
