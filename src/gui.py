#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# Copyright 2018-2019 Artem Smirnov

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# https://pythonworld.ru/gui/pyqt5-firstprograms.html
# https://pythonworld.ru/gui/pyqt5-menustoolbars.html
# https://pythonworld.ru/gui/pyqt5-layout.html

# https://tproger.ru/translations/python-gui-pyqt/
# https://stackoverflow.com/questions/26226730/where-is-qt-designer-app-on-mac-anaconda
# https://nikolak.com/pyqt-qt-designer-getting-started/

# https://build-system.fman.io/qt-designer-download
# https://doc.qt.io/Qt-5/gettingstarted.html#
# https://doc.qt.io/qtcreator/creator-targets.html

import os
import time
import sys  # sys нужен для передачи argv в QApplication
from PyQt5 import QtWidgets
from PyQt5.QtGui import QFont
from PyQt5 import QtCore
from PyQt5.QtWidgets import QMainWindow, QAction, qApp, QApplication

ROOT_PATH = os.path.dirname(__file__)
sys.path.append(os.path.join(ROOT_PATH, '..')) #up a level to get to the settings file
#sys.path.append(os.path.join(ROOT_PATH, '../..')) #up 2 levels to get to the settings file

import ui.MainWindow
import ui.DevicesWindow
import ui.RulesWindow

# open -a Designer
# pyuic5 ./ui/mainwindow.ui -o ./ui/MainWindow.py
# pyuic5 ./ui/deviceswindow.ui -o ./ui/DevicesWindow.py
# pyuic5 ./ui/ruleswindow.ui -o ./ui/RulesWindow.py


class DevicesWindow(QtWidgets.QMainWindow, ui.DevicesWindow.Ui_DevicesWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.pushButton_3.clicked.connect(self.close)


class RulesWindow(QtWidgets.QMainWindow, ui.RulesWindow.Ui_RulesWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.pushButton_2.clicked.connect(self.close) #self.destroy()


class MainWindowApp(QtWidgets.QMainWindow, ui.MainWindow.Ui_MainWindow):

    labels = ["source_path", "size", "hash", "uploaded"]
    key = "source_path"

    def open_rules(self):
        # https://python-scripts.com/question/7159
        # в скобках self -> передаем ссылку на родителя, чтобы окно можно было сделать модальным
        self.window_rules = RulesWindow(self)

        # делаем окно модальным
        self.window_rules.setWindowModality(QtCore.Qt.WindowModal)

        # не совсем понимаю зачем
        # self.window_rules.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.window_rules.show()


    def open_devices(self):
        # https://python-scripts.com/question/7159
        # в скобках self -> передаем ссылку на родителя, чтобы окно можно было сделать модальным
        self.window_devices = DevicesWindow(self)

        # делаем окно модальным
        self.window_devices.setWindowModality(QtCore.Qt.WindowModal)

        # не совсем понимаю зачем
        # self.window_devices.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.window_devices.show()


    def __init__(self):
        # Это здесь нужно для доступа к переменным, методам
        # и т.д. в файле MainWindow.py
        super().__init__()
        self.setupUi(self)  # Это нужно для инициализации нашего дизайна
        self.swapButton.clicked.connect(self.swap_devices)
        self.pushButton_3.clicked.connect(self.open_rules)
        self.init_table()

        self.pushButton_2.setFocus()

        # self.statusbar()
        self.statusbar.showMessage("Ready")

        menubar = self.menuBar()
        settingsMenu = menubar.addMenu("&Settings")

        self.devicesAction = QAction("Devices")
        self.devicesAction.setShortcut("Ctrl+D")
        self.devicesAction.setStatusTip("Manage devices")
        self.devicesAction.triggered.connect(self.open_devices)

        settingsMenu.addAction(self.devicesAction)
        settingsMenu.addAction("Logging")
        settingsMenu.addAction("DB")

        # http://cppstudio.ru/?p=291
        # https://stackoverflow.com/questions/44281357/pyqt-connect-qaction-to-function
        # https://pythonworld.ru/gui/pyqt5-menustoolbars.html
        # https://stackoverflow.com/questions/17065053/how-do-i-use-qss-in-pyqt4
        # https://stackoverflow.com/questions/10024525/howto-draw-correct-css-border-in-header
        try:
            self.setStyleSheet(open("./ui/MainWindow.qss", 'r').read())
        except Exception as ex:
            print(str(ex))


    def add_records(self, records):

        # https://doc.qt.io/qtforpython/PySide2/QtWidgets/QTableWidget.html#PySide2.QtWidgets.PySide2.QtWidgets.QTableWidget.insertRow
        # https://evileg.com/en/post/78/
        # https://secretsilent.ru/добавление-данных-в-таблицу-qtablewidget-очистк/

        for file in records:
            # for label, item in file.iteritems():
            #     # https://pythonworld.ru/tipy-dannyx-v-python/slovari-dict-funkcii-i-metody-slovarej.html
            #     # https://stackoverflow.com/questions/10458437/what-is-the-difference-between-dict-items-and-dict-iteritems
            
            new_row_index = self.tableFiles.rowCount()
            self.tableFiles.insertRow(new_row_index)

            for column_num in range(len(self.labels)):

                newItem = QtWidgets.QTableWidgetItem(1)
                newItem.setText(str(file.get(self.labels[column_num])))

                self.tableFiles.setItem(new_row_index, column_num, newItem)

        self.tableFiles.resizeColumnsToContents()


    def update_record(self, record):
        # check existing of the key & get position of column
        column_num = -1
        for column_num in range(len(self.labels)):
            if self.labels[column_num] == self.key: break
        if column_num < 0: return
        else:
            # search the key in the table
            for row_num in range(self.tableFiles.rowCount()):
                if self.tableFiles.item(row_num, column_num).text() == record[self.key]:

                    # update fields
                    for column_num in range(len(self.labels)):
                        self.tableFiles.item(row_num, column_num).setText(str(record.get(self.labels[column_num])))
                    # resize table
                    self.tableFiles.resizeColumnsToContents()

                    # searching just first occurrence
                    return


    def init_table(self):
        self.tableFiles.clear()  # На случай, если в списке уже есть элементы
        self.tableFiles.setColumnCount(len(self.labels))
        self.tableFiles.setRowCount(0)
        self.tableFiles.setWordWrap(False) # запрет на перенос строк
        self.tableFiles.verticalHeader().setDefaultSectionSize(0)
        self.tableFiles.setHorizontalHeaderLabels(self.labels)

        # https://stackoverflow.com/questions/15686501/qt-qtablewidget-column-resizing
        # from PyQt5.QtWidgets import QHeaderView
        # self.tableFiles.horizontalHeader().setResizeMode(0, QHeaderView.Stretch)

        self.tableFiles.resizeColumnsToContents()

        font = QFont("TypeWriter", pointSize=11, weight=QFont.Medium)
        self.tableFiles.setFont(font)


    def init_source(self, classname, args):
        self.comboBox_Source.clear()
        # http://cppstudio.ru/?p=347
        # https://pythonworld.ru/gui/pyqt5-widgets2.html
        self.comboBox_Source.addItems(["LOCAL: ./temp/source", "DISK: ./temp/source", "FTP", "WEBDAV", "POST"])
        self.comboBox_Source.activated[str].connect(self.onActivated_Source)
        self.comboBox_Source.addItem("classname", 1000)
        # if args['mount_point'] != "": self.lineEdit_Source.setText(str(args['mount_point']))


    def onActivated_Source(self, text):
        pass
        # self.lbl.setText(text)
        # self.lbl.adjustSize()


    def init_target(self, classname, args):
        self.comboBox_Target.clear()
        self.comboBox_Target.addItems(["DISK: /flir", "DISK: ./temp/source", "FTP", "WEBDAV", "POST"])

        # if args['mount_point'] != "": self.lineEdit_Target.setText(str(args['mount_point']))


    def swap_devices(self):
        pass

# directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Выберите папку")
