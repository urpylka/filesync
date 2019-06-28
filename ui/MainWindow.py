# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.12.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 449)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setMinimumSize(QtCore.QSize(800, 400))
        self.centralwidget.setMaximumSize(QtCore.QSize(50000, 50000))
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.comboBox_Source = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_Source.setCurrentText("")
        self.comboBox_Source.setObjectName("comboBox_Source")
        self.horizontalLayout_4.addWidget(self.comboBox_Source)
        self.swapButton = QtWidgets.QPushButton(self.centralwidget)
        self.swapButton.setMinimumSize(QtCore.QSize(45, 0))
        self.swapButton.setMaximumSize(QtCore.QSize(45, 16777215))
        self.swapButton.setObjectName("swapButton")
        self.horizontalLayout_4.addWidget(self.swapButton)
        self.comboBox_Target = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_Target.setObjectName("comboBox_Target")
        self.horizontalLayout_4.addWidget(self.comboBox_Target)
        self.verticalLayout_4.addLayout(self.horizontalLayout_4)
        self.tableFiles = QtWidgets.QTableWidget(self.centralwidget)
        self.tableFiles.setObjectName("tableFiles")
        self.tableFiles.setColumnCount(0)
        self.tableFiles.setRowCount(0)
        self.verticalLayout_4.addWidget(self.tableFiles)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.checkBox_2 = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox_2.setObjectName("checkBox_2")
        self.horizontalLayout.addWidget(self.checkBox_2)
        self.checkBox = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox.setObjectName("checkBox")
        self.horizontalLayout.addWidget(self.checkBox)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Filesync"))
        self.swapButton.setText(_translate("MainWindow", "⇆"))
        self.checkBox_2.setText(_translate("MainWindow", "not before"))
        self.checkBox.setText(_translate("MainWindow", "already equals"))


