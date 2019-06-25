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
import ui.MainWindow  # Это наш конвертированный файл дизайна

# open -a Designer
# pyuic5 ./ui/mainwindow.ui -o ./ui/MainWindow.py


class ExampleApp(QtWidgets.QMainWindow, ui.MainWindow.Ui_MainWindow):
    def __init__(self):
        # Это здесь нужно для доступа к переменным, методам
        # и т.д. в файле MainWindow.py
        super().__init__()
        self.setupUi(self)  # Это нужно для инициализации нашего дизайна
        self.pushButton.clicked.connect(self.browse_folder)


    def browse_folder(self):
        self.listWidget_Source.clear()  # На случай, если в списке уже есть элементы
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Выберите папку")
        # открыть диалог выбора директории и установить значение переменной
        # равной пути к выбранной директории

        if directory:  # не продолжать выполнение, если пользователь не выбрал директорию
            for file_name in os.listdir(directory):  # для каждого файла в директории
                self.listWidget_Source.addItem(file_name)   # добавить файл в listWidget


def main():
    app = QtWidgets.QApplication(sys.argv)  # Новый экземпляр QApplication
    window = ExampleApp()  # Создаём объект класса ExampleApp
    window.show()  # Показываем окно
    app.exec_()  # и запускаем приложение

if __name__ == '__main__':  # Если мы запускаем файл напрямую, а не импортируем
    main()  # то запускаем функцию main()
