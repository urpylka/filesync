# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/deviceswindow.ui'
#
# Created by: PyQt5 UI code generator 5.12.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_DevicesWindow(object):
    def setupUi(self, DevicesWindow):
        DevicesWindow.setObjectName("DevicesWindow")
        DevicesWindow.resize(800, 450)
        DevicesWindow.setMinimumSize(QtCore.QSize(800, 450))
        DevicesWindow.setDockOptions(QtWidgets.QMainWindow.AllowTabbedDocks|QtWidgets.QMainWindow.AnimatedDocks)
        self.centralwidget = QtWidgets.QWidget(DevicesWindow)
        self.centralwidget.setMinimumSize(QtCore.QSize(800, 400))
        self.centralwidget.setMaximumSize(QtCore.QSize(50000, 50000))
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.devicesList = QtWidgets.QListWidget(self.centralwidget)
        self.devicesList.setObjectName("devicesList")
        self.verticalLayout.addWidget(self.devicesList)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.b_Drop = QtWidgets.QPushButton(self.centralwidget)
        self.b_Drop.setObjectName("b_Drop")
        self.horizontalLayout.addWidget(self.b_Drop)
        self.b_Duplicate = QtWidgets.QPushButton(self.centralwidget)
        self.b_Duplicate.setObjectName("b_Duplicate")
        self.horizontalLayout.addWidget(self.b_Duplicate)
        self.b_Add = QtWidgets.QPushButton(self.centralwidget)
        self.b_Add.setObjectName("b_Add")
        self.horizontalLayout.addWidget(self.b_Add)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_3.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.devicesCombo = QtWidgets.QComboBox(self.centralwidget)
        self.devicesCombo.setObjectName("devicesCombo")
        self.verticalLayout_2.addWidget(self.devicesCombo)
        self.deviceArgs = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.deviceArgs.setObjectName("deviceArgs")
        self.verticalLayout_2.addWidget(self.deviceArgs)
        spacerItem = QtWidgets.QSpacerItem(20, 0, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_2.addItem(spacerItem)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.b_Save = QtWidgets.QPushButton(self.centralwidget)
        self.b_Save.setEnabled(False)
        self.b_Save.setCheckable(False)
        self.b_Save.setObjectName("b_Save")
        self.horizontalLayout_2.addWidget(self.b_Save)
        self.b_Cancel = QtWidgets.QPushButton(self.centralwidget)
        self.b_Cancel.setObjectName("b_Cancel")
        self.horizontalLayout_2.addWidget(self.b_Cancel)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3.addLayout(self.verticalLayout_2)
        DevicesWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(DevicesWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        DevicesWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(DevicesWindow)
        self.statusbar.setObjectName("statusbar")
        DevicesWindow.setStatusBar(self.statusbar)

        self.retranslateUi(DevicesWindow)
        QtCore.QMetaObject.connectSlotsByName(DevicesWindow)

    def retranslateUi(self, DevicesWindow):
        _translate = QtCore.QCoreApplication.translate
        DevicesWindow.setWindowTitle(_translate("DevicesWindow", "Devices"))
        self.b_Drop.setText(_translate("DevicesWindow", "Drop"))
        self.b_Duplicate.setText(_translate("DevicesWindow", "Duplicate"))
        self.b_Add.setText(_translate("DevicesWindow", "Add"))
        self.b_Save.setText(_translate("DevicesWindow", "Save"))
        self.b_Cancel.setText(_translate("DevicesWindow", "Cancel"))


