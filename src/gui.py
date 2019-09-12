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
import json
import sys  # sys нужен для передачи argv в QApplication

from PyQt5 import QtWidgets
from PyQt5.QtGui import QFont
from PyQt5 import QtCore
from PyQt5.QtWidgets import QMainWindow, QAction, qApp, QApplication, QMessageBox, QWidget

from threading import Thread
from queue import Queue

from json_array import JsonArray
from logger import get_logger
from main_logic import *

ROOT_PATH = os.path.dirname(__file__)
sys.path.append(os.path.join(ROOT_PATH, '..')) #up a level to get to the settings file
#sys.path.append(os.path.join(ROOT_PATH, '../..')) #up 2 levels to get to the settings file
import ui.MainWindow
import ui.DevicesWindow
import ui.RulesWindow
import ui.LogsWindow

# open -a Designer
# pyuic5 ./ui/mainwindow.ui -o ./ui/MainWindow.py
# pyuic5 ./ui/deviceswindow.ui -o ./ui/DevicesWindow.py
# pyuic5 ./ui/ruleswindow.ui -o ./ui/RulesWindow.py
# pyuic5 ./ui/logswindow.ui -o ./ui/LogsWindow.py


def load_json(json_path):
    """
    Функция для загрузки словаря из json-файла.
    """

    records = []
    try:
        with open(json_path, 'r') as infile:
            records = json.load(infile)

    except IOError as ex:
        if ex.errno == 2:
            print("load_json: File of DB was created")

    except ValueError as ex:
        print("load_json: Incorrect Json in file of DB: " + str(ex))

    return records


def dump_json(json_path, records):
    if not os.path.exists(os.path.dirname(json_path)):
        try:
            os.makedirs(os.path.dirname(json_path))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    with open(json_path, 'w') as outfile:
        json.dump(records, outfile)
        # self._logger.debug("dump_json: File of the DB was updated successful!")


def in_records(key, value, records):
    """
    Функция поиска лога лога в словаре.

    В структуре словаря может отсутствовать атрибут name,
    и нужно сделать отлов ошибки
    AttributeError: 'dict' object has no attribute 'name'
    """
    for record in records:
        if record[key] == value: return True
    return False


class DevicesWindow(QtWidgets.QMainWindow, ui.DevicesWindow.Ui_DevicesWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.parent = parent
        self.b_Cancel.clicked.connect(self.closeDevices)
        self.b_Save.clicked.connect(self.saveDevice)
        self.b_Drop.clicked.connect(self.dropDevice)
        self.b_Duplicate.clicked.connect(self.duplicateDevice)
        self.b_Add.clicked.connect(self.addDevice)

        self.deviceArgs.textChanged.connect(self.argsChanged)

        logger = get_logger("devices", "INFO", True, "")
        self.devices_config = "./devices.json"
        self.devices_array = load_json(self.devices_config)

        self.changed_device = {}

        self.loadDevicesCombo() # must be loaded before devicesList
        self.loadDevicesList()

        self.devicesList.currentRowChanged.connect(self.selectDeviceInList)
        self.devicesCombo.currentIndexChanged.connect(self.selectDeviceInCombo)


    def closeDevices(self):
        self.parent.initDevices()
        self.close()


    def loadDevicesList(self):
        self.devicesList.clear()
        my_list = []
        for dev in self.devices_array:

            cl = dev["device"]
            try:
                d_cl = getattr(__import__(cl), cl).to_string(dev["args"])
                my_list.append(getattr(__import__(cl), cl).to_string(dev["args"]))
            except Exception as ex:
                print(ex)

        self.devicesList.addItems(my_list)

        # Select Row #0 by default
        self.devicesList.setCurrentRow(0)
        self.selectDeviceInList(0)


    def selectDeviceInList(self, index_of_device):
        name_of_device = self.devices_array[index_of_device]["device"]

        index = self.devicesCombo.findText(name_of_device, QtCore.Qt.MatchFixedString)
        if index >= 0: self.devicesCombo.setCurrentIndex(index)

        self.updateParams()


    def loadDevicesCombo(self):
        self.devices_combo = []
        devices_path = "./src"
        for rootdir, dirs, files in os.walk(devices_path):
            for file in files:
                if file.startswith("device_") and file.endswith(".py"): self.devices_combo.append(file[:-3])
        self.devicesCombo.addItems(self.devices_combo)


    def selectDeviceInCombo(self, index_item):
        # print(index_item)
        self.updateParams()


    def updateParams(self):
        device_t = self.devicesCombo.currentText()
        device_c = self.devices_array[self.devicesList.currentRow()]

        try:
            m = __import__(device_t)
            source = getattr(m, device_t).get_fields()

            device_r = {}
            if device_t == device_c["device"]:
                for field in source:
                    if field is not "logger":
                        device_r[field] = "" if device_c["args"][field] is None else device_c["args"][field]
            else:
                for field in source:
                    if field is not "logger": device_r[field] = ""

            # https://python-scripts.com/json
            self.deviceArgs.setPlainText(json.dumps(device_r))
            self.deviceArgs.setReadOnly(False)

            my_device = {}
            my_device["device"] = device_t
            my_device["args"] = device_r

            if my_device != device_c:
                self.changed_device = my_device
                self.b_Save.setEnabled(True)
            else:
                self.b_Save.setEnabled(False)

        except Exception as ex:
            self.deviceArgs.setPlainText("ERROR\n"+str(ex))
            self.deviceArgs.setReadOnly(True)
            # QMessageBox.critical(QWidget(), "ERROR", ex)


    def argsChanged(self):
        device_t = self.devicesCombo.currentText()
        device_c = self.devices_array[self.devicesList.currentRow()]

        my_device = {}
        my_device["device"] = device_t
        my_device["args"] = json.loads(self.deviceArgs.toPlainText())

        if my_device != device_c:
            self.changed_device = my_device
            self.b_Save.setEnabled(True)
        else:
            self.b_Save.setEnabled(False)


    def addDevice(self):

        device_t = self.devicesCombo.currentText()
        device_r = {}

        try:
            m = __import__(device_t)
            source = getattr(m, device_t).get_fields()

            for field in source:
                if field is not "logger":
                    device_r[field] = ""

        except Exception as ex:
            print(ex)

        my_device = {}
        my_device["device"] = device_t
        my_device["args"] = device_r

        self.devices_array.append(my_device)

        dump_json(self.devices_config, self.devices_array)
        self.loadDevicesList()
        self.devicesList.setCurrentRow(self.devicesList.count()-1)


    def duplicateDevice(self):
        self.devices_array.append(self.devices_array[self.devicesList.currentRow()])

        dump_json(self.devices_config, self.devices_array)
        self.loadDevicesList()
        self.devicesList.setCurrentRow(self.devicesList.count()-1)


    def dropDevice(self):
        self.devices_array.pop(self.devicesList.currentRow())
        dump_json(self.devices_config, self.devices_array)
        self.loadDevicesList()


    def saveDevice(self):
        self.devices_array[self.devicesList.currentRow()] = self.changed_device
        dump_json(self.devices_config, self.devices_array)
        self.b_Save.setEnabled(False)
        self.loadDevicesList()


class RulesWindow(QtWidgets.QMainWindow, ui.RulesWindow.Ui_RulesWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.pushButton_2.clicked.connect(self.close) #self.destroy()

        #open config.json
        self.rules_config = "./config.json"
        self.rules_array = load_json(self.rules_config)
        # field Include:
        for i in range(len(self.rules_array["rules"]["include"])):
            self.plainTextEdit.appendPlainText(self.rules_array["rules"]["include"][i])
        #field Exclude:
        for i in range(len(self.rules_array["rules"]["exclude"])):
            self.plainTextEdit_2.appendPlainText(self.rules_array["rules"]["exclude"][i])



class LogsWindow(QtWidgets.QMainWindow, ui.LogsWindow.Ui_LogsWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        # self.pushButton_2.clicked.connect(self.close) #self.destroy()


class MainWindowApp(QtWidgets.QMainWindow, ui.MainWindow.Ui_MainWindow):

    labels = ["source_path", "size", "hash", "uploaded"]
    key = "source_path"


    def __init__(self):
        # Это здесь нужно для доступа к переменным, методам
        # и т.д. в файле MainWindow.py
        super().__init__()
        self.setupUi(self)  # Это нужно для инициализации нашего дизайна

        self.main_config = "./config.json"
        self.main_array = load_json(self.main_config)

        self.devices_config = "./devices.json"
        self.devices_array = []

        self.b_Swap.clicked.connect(self.swapDevices)

        self.cb_Realtime.stateChanged.connect(self.realtime)
        self.cb_Realtime.setChecked(True)

        self.b_Scan.clicked.connect(self.scan)
        self.b_Sync.clicked.connect(self.sync)

        self.initDevices()
        self.initTable()

        menubar = self.menuBar()
        settingsMenu = menubar.addMenu("&Settings")

        self.devicesAction = QAction("Devices")
        self.devicesAction.setShortcut("Ctrl+D")
        self.devicesAction.setStatusTip("Manage devices")
        self.devicesAction.triggered.connect(self.openDevices)
        settingsMenu.addAction(self.devicesAction)

        self.rulesAction = QAction("Rules")
        self.rulesAction.setShortcut("Ctrl+R")
        self.rulesAction.setStatusTip("Manage rules")
        self.rulesAction.triggered.connect(self.openRules)
        settingsMenu.addAction(self.rulesAction)

        self.logsAction = QAction("Logs")
        self.logsAction.setShortcut("Ctrl+L")
        self.logsAction.setStatusTip("View logs")
        self.logsAction.triggered.connect(self.openLogs)
        settingsMenu.addAction(self.logsAction)

        # http://cppstudio.ru/?p=291
        # https://stackoverflow.com/questions/44281357/pyqt-connect-qaction-to-function
        # https://pythonworld.ru/gui/pyqt5-menustoolbars.html
        # https://stackoverflow.com/questions/17065053/how-do-i-use-qss-in-pyqt4
        # https://stackoverflow.com/questions/10024525/howto-draw-correct-css-border-in-header
        # try:
        #     self.setStyleSheet(open("./ui/MainWindow.qss", 'r').read())
        # except Exception as ex:
        #     print(str(ex))

        self.statusbar.showMessage("Ready")


    def realtime(self):
        if self.cb_Realtime.isChecked():
            self.b_Scan.setDisabled(True)
            self.b_Sync.setDisabled(False)
            self.b_Sync.setFocus()
        else:
            self.b_Sync.setDisabled(True)
            self.b_Scan.setDisabled(False)
            self.b_Scan.setFocus()


    def scan(self):
        pass


    def sync(self):
        if self.cb_Realtime.isChecked():

            logger = get_logger(self.main_array["name"], self.main_array["logger"]["log_level"], self.main_array["logger"]["console_output"], self.main_array["logger"]["log_path"])
            db = JsonArray(self.main_array["db_backup"]["db_path"], self.main_array["db_backup"]["autosave_interval"], logger)
            wq = Queue()
            for record in db:
                if not record['downloaded'] or not record['uploaded'] or not record['dropped']: wq.put(record)

            m1 = __import__(self.main_array["source"]["device"])
            self.main_array["source"]["args"]["logger"] = logger
            source = getattr(m1, self.main_array["source"]["device"])(**self.main_array["source"]["args"])

            m2 = __import__(self.main_array["target"]["device"])
            self.main_array["target"]["args"]["logger"] = logger
            target = getattr(m2, self.main_array["target"]["device"])(**self.main_array["target"]["args"])

            def create_threads(count, function, *args):
                for i in range(count):
                    t = Thread(target=function, args=(i+1, args,))
                    t.daemon = True
                    t.start()

            create_threads(1, finder, self, db, source, target, self.main_array["finder"]["search_interval"], self.main_array["finder"]["mkdir_interval"], wq, self.main_array["rules"]["include"], self.main_array["rules"]["exclude"], self.main_array["db"]["key"], self.main_array["db"]["default_record"], logger)
            create_threads(self.main_array["mover_count"], mover, self, target, source, wq, logger)


    def openLogs(self):
        # https://python-scripts.com/question/7159
        # в скобках self -> передаем ссылку на родителя, чтобы окно можно было сделать модальным
        self.window_logs = LogsWindow(self)

        # делаем окно модальным
        # self.window_logs.setWindowModality(QtCore.Qt.WindowModal)

        # не совсем понимаю зачем
        # self.window_rules.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.window_logs.show()


    def openRules(self):
        # https://python-scripts.com/question/7159
        # в скобках self -> передаем ссылку на родителя, чтобы окно можно было сделать модальным
        self.window_rules = RulesWindow(self)

        # делаем окно модальным
        self.window_rules.setWindowModality(QtCore.Qt.WindowModal)

        # не совсем понимаю зачем
        # self.window_rules.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.window_rules.show()


    def openDevices(self):
        # https://python-scripts.com/question/7159
        # в скобках self -> передаем ссылку на родителя, чтобы окно можно было сделать модальным
        self.window_devices = DevicesWindow(self)

        # делаем окно модальным
        self.window_devices.setWindowModality(QtCore.Qt.WindowModal)

        # не совсем понимаю зачем
        # self.window_devices.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.window_devices.show()


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


    def initTable(self):
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


    def initDevices(self):
        self.comboBox_Source.clear()
        self.comboBox_Target.clear()
        # http://cppstudio.ru/?p=347
        # https://pythonworld.ru/gui/pyqt5-widgets2.html

        # Getting list of devices
        self.devices_array = load_json(self.devices_config)

        my_list = []
        for dev in self.devices_array:
            cl = dev["device"]
            try:
                d_cl = getattr(__import__(cl), cl).to_string(dev["args"])
                my_list.append(getattr(__import__(cl), cl).to_string(dev["args"]))
            except Exception as ex:
                print(ex)

        self.comboBox_Source.addItems(my_list)
        self.comboBox_Target.addItems(my_list)

        # Searching positions
        def getPos(arr, item):
            flag = False
            iter = 0
            for item_i in arr:
                if item == item_i:
                    flag = True
                    break
                iter += 1

            return flag, iter

        s_f, s_i = getPos(self.devices_array, self.main_array["source"])
        if s_f: self.comboBox_Source.setCurrentIndex(s_i)
        else:
            self.devices_array.append(self.main_array["source"])
            dump_json(self.devices_config, self.devices_array)
            self.initDevices()

        t_f, t_i = getPos(self.devices_array, self.main_array["target"])
        if t_f: self.comboBox_Target.setCurrentIndex(t_i)
        else:
            self.devices_array.append(self.main_array["target"])
            dump_json(self.devices_config, self.devices_array)
            self.initDevices()

        # self.comboBox_Source.currentIndexChanged.connect(self.onActivated_Source)
        # self.comboBox_Target.currentIndexChanged.connect(self.onActivated_Target)
        self.comboBox_Source.activated[str].connect(self.onActivated_Source)
        self.comboBox_Target.activated[str].connect(self.onActivated_Target)


    def onActivated_Source(self, text):
        index = self.comboBox_Source.findText(text, QtCore.Qt.MatchFixedString)

        self.main_array["source"] = self.devices_array[index]
        dump_json(self.main_config, self.main_array)
        self.initDevices()


    def onActivated_Target(self, text):
        index = self.comboBox_Target.findText(text, QtCore.Qt.MatchFixedString)

        self.main_array["target"] = self.devices_array[index]
        dump_json(self.main_config, self.main_array)
        self.initDevices()


    def swapDevices(self):
        self.main_array["source"] = self.devices_array[self.comboBox_Target.currentIndex()]
        self.main_array["target"] = self.devices_array[self.comboBox_Source.currentIndex()]
        dump_json(self.main_config, self.main_array)
        self.initDevices()

# directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Выберите папку")
