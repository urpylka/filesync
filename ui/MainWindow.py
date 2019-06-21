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
        MainWindow.resize(900, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(220, 80, 114, 32))
        self.pushButton.setObjectName("pushButton")
        self.listWidget_Source = QtWidgets.QListWidget(self.centralwidget)
        self.listWidget_Source.setGeometry(QtCore.QRect(10, 140, 430, 400))
        self.listWidget_Source.setObjectName("listWidget_Source")
        self.listWidget_Target = QtWidgets.QListWidget(self.centralwidget)
        self.listWidget_Target.setGeometry(QtCore.QRect(460, 140, 430, 400))
        self.listWidget_Target.setObjectName("listWidget_Target")
        self.comboBox_Source = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_Source.setGeometry(QtCore.QRect(10, 50, 91, 32))
        self.comboBox_Source.setObjectName("comboBox_Source")
        self.label_Source = QtWidgets.QLabel(self.centralwidget)
        self.label_Source.setGeometry(QtCore.QRect(20, 30, 59, 16))
        self.label_Source.setObjectName("label_Source")
        self.label_Target = QtWidgets.QLabel(self.centralwidget)
        self.label_Target.setGeometry(QtCore.QRect(460, 30, 59, 16))
        self.label_Target.setObjectName("label_Target")
        self.comboBox_Target = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_Target.setGeometry(QtCore.QRect(450, 50, 91, 32))
        self.comboBox_Target.setObjectName("comboBox_Target")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 900, 22))
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
        self.pushButton.setText(_translate("MainWindow", "PushButton"))
        self.label_Source.setText(_translate("MainWindow", "SOURCE"))
        self.label_Target.setText(_translate("MainWindow", "TARGET"))


