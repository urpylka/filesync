# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/logswindow.ui'
#
# Created by: PyQt5 UI code generator 5.12.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_LogsWindow(object):
    def setupUi(self, LogsWindow):
        LogsWindow.setObjectName("LogsWindow")
        LogsWindow.resize(800, 450)
        LogsWindow.setMinimumSize(QtCore.QSize(800, 450))
        self.centralwidget = QtWidgets.QWidget(LogsWindow)
        self.centralwidget.setMinimumSize(QtCore.QSize(800, 400))
        self.centralwidget.setMaximumSize(QtCore.QSize(50000, 50000))
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.plainTextEdit.setEnabled(True)
        self.plainTextEdit.setAcceptDrops(False)
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.verticalLayout.addWidget(self.plainTextEdit)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.b_Clear = QtWidgets.QPushButton(self.centralwidget)
        self.b_Clear.setObjectName("b_Clear")
        self.horizontalLayout_2.addWidget(self.b_Clear)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        LogsWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(LogsWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        LogsWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(LogsWindow)
        self.statusbar.setObjectName("statusbar")
        LogsWindow.setStatusBar(self.statusbar)

        self.retranslateUi(LogsWindow)
        QtCore.QMetaObject.connectSlotsByName(LogsWindow)

    def retranslateUi(self, LogsWindow):
        _translate = QtCore.QCoreApplication.translate
        LogsWindow.setWindowTitle(_translate("LogsWindow", "Rules"))
        self.b_Clear.setText(_translate("LogsWindow", "Clear"))


