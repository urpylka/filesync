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

import sys
import os.path
import time
from threading import Thread
from queue import Queue
from fnmatch import fnmatch 

from json_array import JsonArray
from smart_buffer import SmartBuffer
from mover import Mover
from logger import get_logger
from gui import QtWidgets, MainWindowApp
from main_logic import *


def main():

    if len(sys.argv) != 2:
        print("Error 4. Doesn't have path to the config-file as argument")
        exit(1)
    else:

        config_path = sys.argv[1]
        worker_data = {}
        if os.path.exists(os.path.dirname(config_path)):
            try:
                import json
                with open(config_path, 'r') as infile:
                    worker_data = json.load(infile)

            except IOError as ex:
                if ex.errno == 2:
                    print("Error 1. The config file doesn't exist")
                    exit(1)

            except ValueError as ex:
                print("Error 2. Incorrect Json in the config file: " + str(ex))
                exit(1)
        else:
            print("Error 3. The config file doesn't exist")
            exit(1)

        if worker_data["gui"]:

            # больше одного worker как отдельного процесса вроде как не запустить!

            # https://pythonworld.ru/osnovy/instrukciya-if-elif-else-proverka-istinnosti-trexmestnoe-vyrazhenie-ifelse.html
            # A = Y if X else Z
            app = QtWidgets.QApplication(sys.argv)  # Новый экземпляр QApplication
            global window
            window = MainWindowApp()  # Создаём объект класса ExampleApp

            window.init_source(worker_data["source"]["device"], worker_data["source"]["args"])
            window.init_target(worker_data["target"]["device"], worker_data["target"]["args"])

            window.show()  # Показываем окно

            # create_threads(1, finder, db, source, target, worker_data["finder"]["search_interval"], worker_data["finder"]["mkdir_interval"], wq, worker_data["rules"]["include"], worker_data["rules"]["exclude"], worker_data["db"]["key"], worker_data["db"]["default_record"], logger)
            # create_threads(worker_data["mover_count"], mover, target, source, wq, logger)

            app.exec_()  # и запускаем приложение

            return 0

        else:
            if worker_data["enable"]:
                logger = get_logger(worker_data["name"], worker_data["logger"]["log_level"], worker_data["logger"]["console_output"], worker_data["logger"]["log_path"])

                db = JsonArray(worker_data["db_backup"]["db_path"], worker_data["db_backup"]["autosave_interval"], logger)
                wq = Queue()
                for record in db:
                    if not record['downloaded'] or not record['uploaded'] or not record['dropped']: wq.put(record)

                m1 = __import__(worker_data["source"]["device"])
                worker_data["source"]["args"]["logger"] = logger
                source = getattr(m1, worker_data["source"]["device"])(**worker_data["source"]["args"])

                m2 = __import__(worker_data["target"]["device"])
                worker_data["target"]["args"]["logger"] = logger
                target = getattr(m2, worker_data["target"]["device"])(**worker_data["target"]["args"])

                def create_threads(count, function, *args):
                    for i in range(count):
                        t = Thread(target=function, args=(i+1, args,))
                        t.daemon = True
                        t.start()
                window = None
                create_threads(1, finder, window, db, source, target, worker_data["finder"]["search_interval"], worker_data["finder"]["mkdir_interval"], wq, worker_data["rules"]["include"], worker_data["rules"]["exclude"], worker_data["db"]["key"], worker_data["db"]["default_record"], logger)
                create_threads(worker_data["mover_count"], mover, window, target, source, wq, logger)

            try:
                while True:
                    time.sleep(10)
            except KeyboardInterrupt:
                # del db
                # smart_buffer.dump()
                return 0


if __name__ == '__main__':
    main()
