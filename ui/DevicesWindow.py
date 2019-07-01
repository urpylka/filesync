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
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.listWidget = QtWidgets.QListWidget(self.centralwidget)
        self.listWidget.setObjectName("listWidget")
        self.horizontalLayout.addWidget(self.listWidget)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.horizontalLayout_2.addWidget(self.pushButton_2)
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout_2.addWidget(self.pushButton)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox.setObjectName("comboBox")
        self.verticalLayout_2.addWidget(self.comboBox)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.verticalLayout_2.addWidget(self.lineEdit_3)
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.verticalLayout_2.addWidget(self.lineEdit_2)
        self.lineEdit_4 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.verticalLayout_2.addWidget(self.lineEdit_4)
        self.lineEdit_7 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.verticalLayout_2.addWidget(self.lineEdit_7)
        self.lineEdit_5 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.verticalLayout_2.addWidget(self.lineEdit_5)
        self.lineEdit_6 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.verticalLayout_2.addWidget(self.lineEdit_6)
        self.lineEdit_8 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.verticalLayout_2.addWidget(self.lineEdit_8)
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setObjectName("lineEdit")
        self.verticalLayout_2.addWidget(self.lineEdit)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.verticalLayout_2.addWidget(self.pushButton_3)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.verticalLayout.addLayout(self.horizontalLayout)
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
        self.pushButton_2.setText(_translate("DevicesWindow", "Save"))
        self.pushButton.setText(_translate("DevicesWindow", "Drop"))
        self.pushButton_3.setText(_translate("DevicesWindow", "Cancel"))


