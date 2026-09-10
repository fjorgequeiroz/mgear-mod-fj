import mgear.core.pyqt as gqt
QtGui, QtCore, QtWidgets, wrapInstance = gqt.qt_import()


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(238, 460)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")

        # --- Options group -------------------------------------------------
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setLabelAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing
            | QtCore.Qt.AlignVCenter)
        self.formLayout.setObjectName("formLayout")

        self.divisions_label = QtWidgets.QLabel(self.groupBox)
        self.divisions_label.setObjectName("divisions_label")
        self.formLayout.setWidget(
            0, QtWidgets.QFormLayout.LabelRole, self.divisions_label)
        self.division_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.division_spinBox.setMinimum(3)
        self.division_spinBox.setProperty("value", 5)
        self.division_spinBox.setObjectName("division_spinBox")
        self.formLayout.setWidget(
            0, QtWidgets.QFormLayout.FieldRole, self.division_spinBox)

        self.tangentControls_label = QtWidgets.QLabel(self.groupBox)
        self.tangentControls_label.setObjectName("tangentControls_label")
        self.formLayout.setWidget(
            1, QtWidgets.QFormLayout.LabelRole, self.tangentControls_label)
        self.tangentControls_checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.tangentControls_checkBox.setText("")
        self.tangentControls_checkBox.setObjectName("tangentControls_checkBox")
        self.formLayout.setWidget(
            1, QtWidgets.QFormLayout.FieldRole, self.tangentControls_checkBox)

        self.IKWorldOri_label = QtWidgets.QLabel(self.groupBox)
        self.IKWorldOri_label.setObjectName("IKWorldOri_label")
        self.formLayout.setWidget(
            2, QtWidgets.QFormLayout.LabelRole, self.IKWorldOri_label)
        self.IKWorldOri_checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.IKWorldOri_checkBox.setText("")
        self.IKWorldOri_checkBox.setObjectName("IKWorldOri_checkBox")
        self.formLayout.setWidget(
            2, QtWidgets.QFormLayout.FieldRole, self.IKWorldOri_checkBox)

        self.verticalLayout.addLayout(self.formLayout)
        self.squashStretchProfile_pushButton = QtWidgets.QPushButton(
            self.groupBox)
        self.squashStretchProfile_pushButton.setObjectName(
            "squashStretchProfile_pushButton")
        self.verticalLayout.addWidget(self.squashStretchProfile_pushButton)
        self.gridLayout_2.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 1, 1)

        # --- FK Reference Array group ------------------------------------
        self.fkRefArray_groupBox = QtWidgets.QGroupBox(Form)
        self.fkRefArray_groupBox.setObjectName("fkRefArray_groupBox")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.fkRefArray_groupBox)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.fkRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.fkRefArray_horizontalLayout.setObjectName(
            "fkRefArray_horizontalLayout")
        self.fkRefArray_verticalLayout_1 = QtWidgets.QVBoxLayout()
        self.fkRefArray_verticalLayout_1.setObjectName(
            "fkRefArray_verticalLayout_1")
        self.fkRefArray_listWidget = QtWidgets.QListWidget(
            self.fkRefArray_groupBox)
        self.fkRefArray_listWidget.setDragDropOverwriteMode(True)
        self.fkRefArray_listWidget.setDragDropMode(
            QtWidgets.QAbstractItemView.InternalMove)
        self.fkRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.fkRefArray_listWidget.setAlternatingRowColors(True)
        self.fkRefArray_listWidget.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection)
        self.fkRefArray_listWidget.setSelectionRectVisible(False)
        self.fkRefArray_listWidget.setObjectName("fkRefArray_listWidget")
        self.fkRefArray_verticalLayout_1.addWidget(self.fkRefArray_listWidget)
        self.fkRefArray_copyRef_pushButton = QtWidgets.QPushButton(
            self.fkRefArray_groupBox)
        self.fkRefArray_copyRef_pushButton.setObjectName(
            "fkRefArray_copyRef_pushButton")
        self.fkRefArray_verticalLayout_1.addWidget(
            self.fkRefArray_copyRef_pushButton)
        self.fkRefArray_horizontalLayout.addLayout(
            self.fkRefArray_verticalLayout_1)
        self.fkRefArray_verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.fkRefArray_verticalLayout_2.setObjectName(
            "fkRefArray_verticalLayout_2")
        self.fkRefArrayAdd_pushButton = QtWidgets.QPushButton(
            self.fkRefArray_groupBox)
        self.fkRefArrayAdd_pushButton.setObjectName("fkRefArrayAdd_pushButton")
        self.fkRefArray_verticalLayout_2.addWidget(
            self.fkRefArrayAdd_pushButton)
        self.fkRefArrayRemove_pushButton = QtWidgets.QPushButton(
            self.fkRefArray_groupBox)
        self.fkRefArrayRemove_pushButton.setObjectName(
            "fkRefArrayRemove_pushButton")
        self.fkRefArray_verticalLayout_2.addWidget(
            self.fkRefArrayRemove_pushButton)
        spacerItem = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum,
            QtWidgets.QSizePolicy.Expanding)
        self.fkRefArray_verticalLayout_2.addItem(spacerItem)
        self.fkRefArray_horizontalLayout.addLayout(
            self.fkRefArray_verticalLayout_2)
        self.gridLayout_3.addLayout(
            self.fkRefArray_horizontalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.fkRefArray_groupBox, 1, 0, 1, 1)

        # --- Head Reference Array group ---------------------------------
        self.headRefArray_groupBox = QtWidgets.QGroupBox(Form)
        self.headRefArray_groupBox.setObjectName("headRefArray_groupBox")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.headRefArray_groupBox)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.headRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.headRefArray_horizontalLayout.setObjectName(
            "headRefArray_horizontalLayout")
        self.headRefArray_verticalLayout_1 = QtWidgets.QVBoxLayout()
        self.headRefArray_verticalLayout_1.setObjectName(
            "headRefArray_verticalLayout_1")
        self.headRefArray_listWidget = QtWidgets.QListWidget(
            self.headRefArray_groupBox)
        self.headRefArray_listWidget.setDragDropOverwriteMode(True)
        self.headRefArray_listWidget.setDragDropMode(
            QtWidgets.QAbstractItemView.InternalMove)
        self.headRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.headRefArray_listWidget.setAlternatingRowColors(True)
        self.headRefArray_listWidget.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection)
        self.headRefArray_listWidget.setSelectionRectVisible(False)
        self.headRefArray_listWidget.setObjectName("headRefArray_listWidget")
        self.headRefArray_verticalLayout_1.addWidget(
            self.headRefArray_listWidget)
        self.headRefArray_copyRef_pushButton = QtWidgets.QPushButton(
            self.headRefArray_groupBox)
        self.headRefArray_copyRef_pushButton.setObjectName(
            "headRefArray_copyRef_pushButton")
        self.headRefArray_verticalLayout_1.addWidget(
            self.headRefArray_copyRef_pushButton)
        self.headRefArray_horizontalLayout.addLayout(
            self.headRefArray_verticalLayout_1)
        self.headRefArray_verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.headRefArray_verticalLayout_2.setObjectName(
            "headRefArray_verticalLayout_2")
        self.headRefArrayAdd_pushButton = QtWidgets.QPushButton(
            self.headRefArray_groupBox)
        self.headRefArrayAdd_pushButton.setObjectName(
            "headRefArrayAdd_pushButton")
        self.headRefArray_verticalLayout_2.addWidget(
            self.headRefArrayAdd_pushButton)
        self.headRefArrayRemove_pushButton = QtWidgets.QPushButton(
            self.headRefArray_groupBox)
        self.headRefArrayRemove_pushButton.setObjectName(
            "headRefArrayRemove_pushButton")
        self.headRefArray_verticalLayout_2.addWidget(
            self.headRefArrayRemove_pushButton)
        spacerItem1 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum,
            QtWidgets.QSizePolicy.Expanding)
        self.headRefArray_verticalLayout_2.addItem(spacerItem1)
        self.headRefArray_horizontalLayout.addLayout(
            self.headRefArray_verticalLayout_2)
        self.gridLayout_4.addLayout(
            self.headRefArray_horizontalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.headRefArray_groupBox, 2, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(gqt.fakeTranslate("Form", "Form", None, -1))
        self.divisions_label.setText(
            gqt.fakeTranslate("Form", "Divisions", None, -1))
        self.tangentControls_label.setText(
            gqt.fakeTranslate("Form", "Tangent Controls", None, -1))
        self.IKWorldOri_checkBox.setToolTip(
            gqt.fakeTranslate(
                "Form",
                "If checked, the IK read-out control will be aligned to "
                "world space",
                None, -1))
        self.IKWorldOri_label.setText(
            gqt.fakeTranslate("Form", "IK Ctl World Ori", None, -1))
        self.squashStretchProfile_pushButton.setText(
            gqt.fakeTranslate("Form", "Squash and Stretch Profile", None, -1))
        self.fkRefArray_groupBox.setTitle(
            gqt.fakeTranslate("Form", "FK Base Reference Array", None, -1))
        self.fkRefArray_copyRef_pushButton.setText(
            gqt.fakeTranslate("Form", "Copy from Head Ref", None, -1))
        self.fkRefArrayAdd_pushButton.setText(
            gqt.fakeTranslate("Form", "<<", None, -1))
        self.fkRefArrayRemove_pushButton.setText(
            gqt.fakeTranslate("Form", ">>", None, -1))
        self.headRefArray_groupBox.setTitle(
            gqt.fakeTranslate("Form", "Head Reference Array", None, -1))
        self.headRefArray_copyRef_pushButton.setText(
            gqt.fakeTranslate("Form", "Copy from FK Ref", None, -1))
        self.headRefArrayAdd_pushButton.setText(
            gqt.fakeTranslate("Form", "<<", None, -1))
        self.headRefArrayRemove_pushButton.setText(
            gqt.fakeTranslate("Form", ">>", None, -1))
