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


def try_again(times, interval, error_message_func, func, *args):
    """
    The function is for trying to execute smthn
    """

    if times < 0:
        raise Exception("Times argument must be positive or zero")
    elif times > 0:
        for iter in range(times):
            try:
                func(*args)
                return
            except Exception as ex:
                error_message_func(ex)
                time.sleep(interval)
        raise Exception("Max iteration exceeding")
    else:
        while True:
            try:
                func(*args)
                return
            except Exception as ex:
                error_message_func(ex)
                time.sleep(interval)


def is_filtered(filename, include=[], exclude=[]):
    """
    Use times = 0 for execute until it will be works!

    The function based on article:
    https://ru.stackoverflow.com/questions/463862/Найти-файлы-по-части-пути-не-только-по-имени-c-python

    Вообще можно фильтровать в теории
    (если решить вопрос с запросом всех данных сразу)
    по след. критериям:
    - min_filesize
    - max_filesize
    - created_interval (start, end)
    - modified_interval (start, end)

    или может прям добавить алгоритм, типа:
    min_filesize = 120 && begin_created = 20190520 || end_modified = 20150811
    """
    for excl in exclude:
        if fnmatch(filename, excl):
            return True
    for incl in include:
        if fnmatch(filename, incl):
            return False
    return True


def finder(number, args):
    """
    Смысла в нескольких finder нет, если только не запросы паралельного листинга,
    что в случае с текущим алгоритмом исключено.

    Или, возможно, это будет полезно для запрашивания инфы
    """

    db, source, target, search_interval, mkdir_interval, wq, include, exclude, logger, db_key, db_def_record = args
    logger.debug("Finder-" + str(number) + " was created.")

    while True:
        try:
            dirs = Queue()
            dirs.put('/')

            while True:

                # For waiting if had smthn in db_backup
                # and for waiting worker on cur_dir
                while not wq.empty():
                    time.sleep(1)

                if not dirs.empty():
                    cur_dir = dirs.get()

                    # Creating directory on target if it doesn't exist
                    try_again(0, mkdir_interval, logger.info, target.mkdir, cur_dir)

                    # Searching for files & dirs
                    for file in source.ls(cur_dir):
                        file = os.path.join(cur_dir, file)
                        
                        if is_filtered(file, include, exclude):
                            logger.debug("Filtred: " + str(file))
                        else:
                            if source.is_dir(file):
                                dirs.put(file)
                            else:
                                
                                if not db.in_records(db_key, file):

                                    # Prepare new object
                                    record = db_def_record.copy()
                                    record[db_key] = file

                                    # Save the object
                                    db.append(record)
                                    wq.put(record)
                                    logger.info("Finder-{0}: Found a new file: {1}".format(str(number), str(file)))

                    dirs.task_done()

        except Exception as ex:
            logger.error("Finder-" + str(number) + ": " + str(ex))
        time.sleep(search_interval)


def worker(number, args):

    target, source, wq, logger = args

    while True:
        # Object from queue takes by link
        # therefore changes the record gives changes in JsonArray
        element = wq.get()

        # Assigment filesize to db
        # element['size'] = source.get_size(element['source_path'])
        def f(elem_p): element['size'] = source.get_size(elem_p)            
        try_again(0, 1, logger.info, f, element['source_path'])

        m = Mover(logger, source, target, element, number)
        m.move()

        wq.task_done()


def main():

    if len(sys.argv) != 2:
        print("Error 4. Doesn't have path to the config-file as argument")
        exit(1)
    else:

        config_path = sys.argv[1]
        config = {}
        if os.path.exists(os.path.dirname(config_path)):
            try:
                import json
                with open(config_path, 'r') as infile:
                    config = json.load(infile)

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


        if len(config["workers"]) < 1:
            print("Error 4. Count of the worker less then 1")
            exit(1)
        else:
            for worker_data in config["workers"]:
                if worker_data["enable"]:
                    logger = get_logger(worker_data["name"], worker_data["logger"]["log_path"], worker_data["logger"]["log_level"])

                    db = JsonArray(worker_data["db_backup"]["db_path"], worker_data["db_backup"]["autosave_interval"], logger)
                    wq = Queue()
                    for record in db:
                        if not record['downloaded'] or not record['uploaded'] or not record['dropped']: wq.put(record)

                    m1 = __import__(worker_data["source"]["module_path"])
                    worker_data["source"]["args"]["logger"] = logger
                    source = getattr(m1, worker_data["source"]["device_class"])(**worker_data["source"]["args"])

                    m2 = __import__(worker_data["target"]["module_path"])
                    worker_data["target"]["args"]["logger"] = logger
                    target = getattr(m2, worker_data["target"]["device_class"])(**worker_data["target"]["args"])

                    def create_threads(count, function, *args):
                        for i in range(count):
                            t = Thread(target=function, args=(i+1, args,))
                            t.daemon = True
                            t.start()

                    create_threads(1, finder, db, source, worker_data["finder"]["finder_interval"], wq, worker_data["finder"]["extensions"], logger)
                    create_threads(worker_data["count"], worker, target, source, wq, logger)

        try:
            while True:
                time.sleep(10)
        except KeyboardInterrupt:
            # del db
            # smart_buffer.dump()
            return 0


if __name__ == '__main__':
    main()
