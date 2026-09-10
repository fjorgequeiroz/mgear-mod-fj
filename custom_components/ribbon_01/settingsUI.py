import mgear.core.pyqt as gqt
QtGui, QtCore, QtWidgets, wrapInstance = gqt.qt_import()


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(238, 320)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")

        # --- Counts group ------------------------------------------------
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setLabelAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing
            | QtCore.Qt.AlignVCenter)
        self.formLayout.setObjectName("formLayout")

        self.fkNb_label = QtWidgets.QLabel(self.groupBox)
        self.fkNb_label.setObjectName("fkNb_label")
        self.formLayout.setWidget(
            0, QtWidgets.QFormLayout.LabelRole, self.fkNb_label)
        self.fkNb_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.fkNb_spinBox.setMinimum(2)
        self.fkNb_spinBox.setMaximum(999)
        self.fkNb_spinBox.setProperty("value", 3)
        self.fkNb_spinBox.setObjectName("fkNb_spinBox")
        self.formLayout.setWidget(
            0, QtWidgets.QFormLayout.FieldRole, self.fkNb_spinBox)

        self.jntNb_label = QtWidgets.QLabel(self.groupBox)
        self.jntNb_label.setObjectName("jntNb_label")
        self.formLayout.setWidget(
            1, QtWidgets.QFormLayout.LabelRole, self.jntNb_label)
        self.jntNb_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.jntNb_spinBox.setMinimum(2)
        self.jntNb_spinBox.setMaximum(999)
        self.jntNb_spinBox.setProperty("value", 10)
        self.jntNb_spinBox.setObjectName("jntNb_spinBox")
        self.formLayout.setWidget(
            1, QtWidgets.QFormLayout.FieldRole, self.jntNb_spinBox)

        self.tweakControls_label = QtWidgets.QLabel(self.groupBox)
        self.tweakControls_label.setObjectName("tweakControls_label")
        self.formLayout.setWidget(
            2, QtWidgets.QFormLayout.LabelRole, self.tweakControls_label)
        self.tweakControls_checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.tweakControls_checkBox.setText("")
        self.tweakControls_checkBox.setObjectName("tweakControls_checkBox")
        self.formLayout.setWidget(
            2, QtWidgets.QFormLayout.FieldRole, self.tweakControls_checkBox)

        self.gridLayout_2.addLayout(self.formLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 1, 1)

        # --- Base Reference Array group --------------------------------
        self.ikRefArray_groupBox = QtWidgets.QGroupBox(Form)
        self.ikRefArray_groupBox.setObjectName("ikRefArray_groupBox")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.ikRefArray_groupBox)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.ikRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.ikRefArray_horizontalLayout.setObjectName(
            "ikRefArray_horizontalLayout")
        self.ikRefArray_verticalLayout_1 = QtWidgets.QVBoxLayout()
        self.ikRefArray_verticalLayout_1.setObjectName(
            "ikRefArray_verticalLayout_1")
        self.ikRefArray_listWidget = QtWidgets.QListWidget(
            self.ikRefArray_groupBox)
        self.ikRefArray_listWidget.setDragDropOverwriteMode(True)
        self.ikRefArray_listWidget.setDragDropMode(
            QtWidgets.QAbstractItemView.InternalMove)
        self.ikRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.ikRefArray_listWidget.setAlternatingRowColors(True)
        self.ikRefArray_listWidget.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection)
        self.ikRefArray_listWidget.setSelectionRectVisible(False)
        self.ikRefArray_listWidget.setObjectName("ikRefArray_listWidget")
        self.ikRefArray_verticalLayout_1.addWidget(self.ikRefArray_listWidget)
        self.ikRefArray_horizontalLayout.addLayout(
            self.ikRefArray_verticalLayout_1)
        self.ikRefArray_verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.ikRefArray_verticalLayout_2.setObjectName(
            "ikRefArray_verticalLayout_2")
        self.ikRefArrayAdd_pushButton = QtWidgets.QPushButton(
            self.ikRefArray_groupBox)
        self.ikRefArrayAdd_pushButton.setObjectName("ikRefArrayAdd_pushButton")
        self.ikRefArray_verticalLayout_2.addWidget(
            self.ikRefArrayAdd_pushButton)
        self.ikRefArrayRemove_pushButton = QtWidgets.QPushButton(
            self.ikRefArray_groupBox)
        self.ikRefArrayRemove_pushButton.setObjectName(
            "ikRefArrayRemove_pushButton")
        self.ikRefArray_verticalLayout_2.addWidget(
            self.ikRefArrayRemove_pushButton)
        spacerItem = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum,
            QtWidgets.QSizePolicy.Expanding)
        self.ikRefArray_verticalLayout_2.addItem(spacerItem)
        self.ikRefArray_horizontalLayout.addLayout(
            self.ikRefArray_verticalLayout_2)
        self.gridLayout_3.addLayout(
            self.ikRefArray_horizontalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.ikRefArray_groupBox, 1, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(gqt.fakeTranslate("Form", "Form", None, -1))
        self.fkNb_label.setText(
            gqt.fakeTranslate("Form", "FK Controllers", None, -1))
        self.jntNb_label.setText(
            gqt.fakeTranslate("Form", "Deform Joints", None, -1))
        self.tweakControls_label.setText(
            gqt.fakeTranslate("Form", "Tweak Controls", None, -1))
        self.ikRefArray_groupBox.setTitle(
            gqt.fakeTranslate("Form", "Base Reference Array", None, -1))
        self.ikRefArrayAdd_pushButton.setText(
            gqt.fakeTranslate("Form", "<<", None, -1))
        self.ikRefArrayRemove_pushButton.setText(
            gqt.fakeTranslate("Form", ">>", None, -1))
