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
import sys  # sys нужен для передачи argv в QApplication
from PyQt5 import QtWidgets
from PyQt5.QtGui import QFont

ROOT_PATH = os.path.dirname(__file__)
sys.path.append(os.path.join(ROOT_PATH, '..')) #up a level to get to the settings file
#sys.path.append(os.path.join(ROOT_PATH, '../..')) #up 2 levels to get to the settings file
import ui.MainWindow  # Это наш конвертированный файл дизайна

# open -a Designer
# pyuic5 ./ui/mainwindow.ui -o ./ui/MainWindow.py


class MainWindowApp(QtWidgets.QMainWindow, ui.MainWindow.Ui_MainWindow):
    def __init__(self):
        # Это здесь нужно для доступа к переменным, методам
        # и т.д. в файле MainWindow.py
        super().__init__()
        self.setupUi(self)  # Это нужно для инициализации нашего дизайна
        self.pushButton.clicked.connect(self.add_elem)
        self.init_table()
        self.add_elem()

        # https://stackoverflow.com/questions/17065053/how-do-i-use-qss-in-pyqt4
        # https://stackoverflow.com/questions/10024525/howto-draw-correct-css-border-in-header
        try:
            self.setStyleSheet(open("./ui/MainWindow.qss", 'r').read())
        except Exception as ex:
            print(str(ex))

    def add_elem(self):
        # https://doc.qt.io/qtforpython/PySide2/QtWidgets/QTableWidget.html#PySide2.QtWidgets.PySide2.QtWidgets.QTableWidget.insertRow
        # https://evileg.com/en/post/78/
        # https://secretsilent.ru/добавление-данных-в-таблицу-qtablewidget-очистк/
        for file_name in os.listdir('.'):  # для каждого файла в директории
            newItem = QtWidgets.QTableWidgetItem(1)
            newItem.setText(file_name)
            font = QFont("TypeWriter", pointSize=11, weight=QFont.Medium)
            newItem.setFont(font)

            new_row_index = self.tableFiles.rowCount()
            self.tableFiles.insertRow(new_row_index)
            self.tableFiles.setItem(new_row_index, 0, newItem)
        
        self.tableFiles.resizeColumnsToContents()

    def browse_folder(self):
        self.listWidget_Source.clear()  # На случай, если в списке уже есть элементы
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Выберите папку")
        # открыть диалог выбора директории и установить значение переменной
        # равной пути к выбранной директории

        if directory:  # не продолжать выполнение, если пользователь не выбрал директорию
            for file_name in os.listdir(directory):  # для каждого файла в директории
                self.listWidget_Source.addItem(file_name)   # добавить файл в listWidget


    def init_table(self):
        self.tableFiles.clear()  # На случай, если в списке уже есть элементы
        self.tableFiles.setColumnCount(5)
        self.tableFiles.setRowCount(0)
        self.tableFiles.setWordWrap(False) # запрет на перенос строк
        self.tableFiles.verticalHeader().setDefaultSectionSize(0)
        self.tableFiles.setHorizontalHeaderLabels(["source_path","size","hash","progress","checkbox"])
        self.tableFiles.resizeColumnsToContents()
        self.tableFiles


        # newItem = QtWidgets.QTableWidgetItem(1)
        # newItem.setText("urpylka")
        # # newItem.setTextAlignment(QtWidgets.Qt) #Qt::AlignCenter
        # self.tableFiles.setItem(1, 2, newItem)


        # // выделяем память под все ячейки таблицы
        # for row = 0; row < tableWidget->rowCount(); row++:
        # for(int column = 0; column < tableWidget->columnCount(); column++)
        # {
        #     QTableWidgetItem *item = new QTableWidgetItem(); // выделяем память под ячейку
        #     item->setText(QString("%1_%2").arg(row).arg(column)); // вставляем текст
    
        #     tableWidget->setItem(row, column, item); // вставляем ячейку
        # }

        # directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Выберите папку")
        # # открыть диалог выбора директории и установить значение переменной
        # # равной пути к выбранной директории

        # if directory:  # не продолжать выполнение, если пользователь не выбрал директорию
        #     for file_name in os.listdir(directory):  # для каждого файла в директории
        #         self.listWidget_Source.addItem(file_name)   # добавить файл в listWidget
