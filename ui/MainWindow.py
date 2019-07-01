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
        MainWindow.resize(800, 450)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setMinimumSize(QtCore.QSize(800, 400))
        self.centralwidget.setMaximumSize(QtCore.QSize(50000, 50000))
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_3.setContentsMargins(12, -1, -1, 12)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
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
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(-1, 12, -1, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.pushButton_2.setObjectName("pushButton_2")
        self.horizontalLayout.addWidget(self.pushButton_2)
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.pushButton_3.setObjectName("pushButton_3")
        self.horizontalLayout.addWidget(self.pushButton_3)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.tableFiles = QtWidgets.QTableWidget(self.centralwidget)
        self.tableFiles.setObjectName("tableFiles")
        self.tableFiles.setColumnCount(0)
        self.tableFiles.setRowCount(0)
        self.verticalLayout_3.addWidget(self.tableFiles)
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
        self.pushButton.setText(_translate("MainWindow", "Sync"))
        self.pushButton_2.setText(_translate("MainWindow", "Scan"))
        self.pushButton_3.setText(_translate("MainWindow", "Settings"))


