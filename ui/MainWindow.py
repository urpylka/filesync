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
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setSpacing(12)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.comboBox_Source = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_Source.setCurrentText("")
        self.comboBox_Source.setObjectName("comboBox_Source")
        self.horizontalLayout_4.addWidget(self.comboBox_Source)
        self.b_Swap = QtWidgets.QPushButton(self.centralwidget)
        self.b_Swap.setMinimumSize(QtCore.QSize(45, 0))
        self.b_Swap.setMaximumSize(QtCore.QSize(45, 16777215))
        self.b_Swap.setObjectName("b_Swap")
        self.horizontalLayout_4.addWidget(self.b_Swap)
        self.comboBox_Target = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_Target.setObjectName("comboBox_Target")
        self.horizontalLayout_4.addWidget(self.comboBox_Target)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(-1, 0, -1, 12)
        self.horizontalLayout.setSpacing(12)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.b_Scan = QtWidgets.QPushButton(self.centralwidget)
        self.b_Scan.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.b_Scan.setCheckable(True)
        self.b_Scan.setAutoDefault(False)
        self.b_Scan.setDefault(False)
        self.b_Scan.setObjectName("b_Scan")
        self.horizontalLayout.addWidget(self.b_Scan)
        self.cb_Realtime = QtWidgets.QCheckBox(self.centralwidget)
        self.cb_Realtime.setObjectName("cb_Realtime")
        self.horizontalLayout.addWidget(self.cb_Realtime)
        self.b_Sync = QtWidgets.QPushButton(self.centralwidget)
        self.b_Sync.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.b_Sync.setCheckable(True)
        self.b_Sync.setObjectName("b_Sync")
        self.horizontalLayout.addWidget(self.b_Sync)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.tableFiles = QtWidgets.QTableWidget(self.centralwidget)
        self.tableFiles.setObjectName("tableFiles")
        self.tableFiles.setColumnCount(0)
        self.tableFiles.setRowCount(0)
        self.verticalLayout.addWidget(self.tableFiles)
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
        self.b_Swap.setText(_translate("MainWindow", "⇆"))
        self.b_Scan.setText(_translate("MainWindow", "Scan"))
        self.cb_Realtime.setText(_translate("MainWindow", "Realtime"))
        self.b_Sync.setText(_translate("MainWindow", "Sync"))


