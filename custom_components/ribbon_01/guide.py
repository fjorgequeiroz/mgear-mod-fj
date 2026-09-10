"""Guide Ribbon 01 module"""

from functools import partial

from mgear.shifter.component import guide
from mgear.core import pyqt
from mgear.vendor.Qt import QtWidgets, QtCore

from maya.app.general.mayaMixin import MayaQWidgetDockableMixin
from maya.app.general.mayaMixin import MayaQDockWidget

from . import settingsUI as sui

# guide info
AUTHOR = "mGear"
URL = ""
EMAIL = ""
VERSION = [1, 0, 0]
TYPE = "ribbon_01"
NAME = "ribbon"
DESCRIPTION = (
    "Ribbon / surface rig. A NURBS surface is skinned to a chain of bind "
    "joints blended between an FK chain and an IK-spline solve (Mode: FK / "
    "IK / FK-IK). Any number of deform joints ride the surface. Twist, "
    "roll and volume preservation on the UI host."
)

##########################################################
# CLASS
##########################################################


class Guide(guide.ComponentGuide):
    """Component Guide Class"""

    compType = TYPE
    compName = NAME
    description = DESCRIPTION

    author = AUTHOR
    url = URL
    email = EMAIL
    version = VERSION

    def postInit(self):
        """Initialize the position for the guide"""
        self.save_transform = ["root", "#_loc"]
        self.save_blade = ["blade"]
        self.addMinMax("#_loc", 1, -1)

    def addObjects(self):
        """Add the Guide Root, blade and locators"""

        self.root = self.addRoot()
        self.locs = self.addLocMulti("#_loc", self.root)
        self.blade = self.addBlade("blade", self.root, self.locs[0])

        centers = [self.root]
        centers.extend(self.locs)
        self.dispcrv = self.addDispCurve("crv", centers)
        self.addDispCurve("crvRef", centers, 3)

    def addParameters(self):
        """Add the configurations settings"""

        # Mode: 0 FK, 1 IK, 2 FK/IK
        self.pMode = self.addParam("mode", "long", 2, 0, 2)
        self.pBlend = self.addParam("blend", "double", 1, 0, 1)

        # Counts
        self.pFkNb = self.addParam("fkNb", "long", 3, 2)
        self.pJntNb = self.addParam("jntNb", "long", 10, 2)

        # Options
        self.pTweakControls = self.addParam(
            "tweakControls", "bool", False)

        # Ref array
        self.pIkRefArray = self.addParam("ikrefarray", "string", "")

        self.pUseIndex = self.addParam("useIndex", "bool", False)
        self.pParentJointIndex = self.addParam(
            "parentJointIndex", "long", -1, None, None)

    def get_divisions(self):
        """Returns correct segments divisions"""
        self.divisions = self.root.jntNb.get()
        return self.divisions


##########################################################
# Setting Page
##########################################################


class settingsTab(QtWidgets.QDialog, sui.Ui_Form):
    """The Component settings UI"""

    def __init__(self, parent=None):
        super(settingsTab, self).__init__(parent)
        self.setupUi(self)


class componentSettings(MayaQWidgetDockableMixin, guide.componentMainSettings):
    """Create the component setting window"""

    def __init__(self, parent=None):
        self.toolName = TYPE
        pyqt.deleteInstances(self, MayaQDockWidget)

        super(componentSettings, self).__init__(parent=parent)
        self.settingsTab = settingsTab()

        self.setup_componentSettingWindow()
        self.create_componentControls()
        self.populate_componentControls()
        self.create_componentLayout()
        self.create_componentConnections()

    def setup_componentSettingWindow(self):
        self.mayaMainWindow = pyqt.maya_main_window()

        self.setObjectName(self.toolName)
        self.setWindowFlags(QtCore.Qt.Window)
        self.setWindowTitle(TYPE)
        self.resize(350, 340)

    def create_componentControls(self):
        return

    def populate_componentControls(self):
        """Populate the controls values from the custom attributes."""
        self.tabs.insertTab(1, self.settingsTab, "Component Settings")

        self.settingsTab.mode_comboBox.setCurrentIndex(
            self.root.attr("mode").get())
        self.settingsTab.blend_spinBox.setValue(
            self.root.attr("blend").get())
        self.settingsTab.fkNb_spinBox.setValue(
            self.root.attr("fkNb").get())
        self.settingsTab.jntNb_spinBox.setValue(
            self.root.attr("jntNb").get())

        self.populateCheck(self.settingsTab.tweakControls_checkBox,
                           "tweakControls")

        ikRefArrayItems = self.root.attr("ikrefarray").get().split(",")
        for item in ikRefArrayItems:
            self.settingsTab.ikRefArray_listWidget.addItem(item)

    def create_componentLayout(self):
        self.settings_layout = QtWidgets.QVBoxLayout()
        self.settings_layout.addWidget(self.tabs)
        self.settings_layout.addWidget(self.close_button)
        self.setLayout(self.settings_layout)

    def create_componentConnections(self):
        self.settingsTab.mode_comboBox.currentIndexChanged.connect(
            partial(self.updateComboBox,
                    self.settingsTab.mode_comboBox,
                    "mode"))

        self.settingsTab.blend_spinBox.valueChanged.connect(
            partial(self.updateSpinBox,
                    self.settingsTab.blend_spinBox,
                    "blend"))

        self.settingsTab.fkNb_spinBox.valueChanged.connect(
            partial(self.updateSpinBox,
                    self.settingsTab.fkNb_spinBox,
                    "fkNb"))

        self.settingsTab.jntNb_spinBox.valueChanged.connect(
            partial(self.updateSpinBox,
                    self.settingsTab.jntNb_spinBox,
                    "jntNb"))

        self.settingsTab.tweakControls_checkBox.stateChanged.connect(
            partial(self.updateCheck,
                    self.settingsTab.tweakControls_checkBox,
                    "tweakControls"))

        self.settingsTab.ikRefArrayAdd_pushButton.clicked.connect(
            partial(self.addItem2listWidget,
                    self.settingsTab.ikRefArray_listWidget,
                    "ikrefarray"))

        self.settingsTab.ikRefArrayRemove_pushButton.clicked.connect(
            partial(self.removeSelectedFromListWidget,
                    self.settingsTab.ikRefArray_listWidget,
                    "ikrefarray"))

        self.settingsTab.ikRefArray_listWidget.installEventFilter(self)

    def eventFilter(self, sender, event):
        if event.type() == QtCore.QEvent.ChildRemoved:
            if sender == self.settingsTab.ikRefArray_listWidget:
                self.updateListAttr(sender, "ikrefarray")
            return True
        else:
            return QtWidgets.QDialog.eventFilter(self, sender, event)

    def dockCloseEventTriggered(self):
        pyqt.deleteInstances(self, MayaQDockWidget)
